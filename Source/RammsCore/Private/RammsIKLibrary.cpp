#include "RammsIKLibrary.h"

#include "Math/UnrealMathUtility.h"
#include "Logging/LogMacros.h"
#include <Eigen/Dense>

static FVector SafeNormalAxis(const FVector& V)
{
	const double S = V.Size();
	if (S <= KINDA_SMALL_NUMBER)
		return FVector(1, 0, 0);
	return V / S;
}

static Eigen::Vector3d ToEigen(const FVector& V) { return {V.X, V.Y, V.Z}; }

static Eigen::Vector3d RotationErrorAxisAngleRad(const FQuat& Current, const FQuat& Target)
{
	// q_err maps current -> target
	FQuat Qerr = Target * Current.Inverse();
	Qerr.Normalize();

	// shortest path
	if (Qerr.W < 0.0f)
	{
		Qerr.X *= -1.0f; Qerr.Y *= -1.0f; Qerr.Z *= -1.0f; Qerr.W *= -1.0f;
	}

	const double w = FMath::Clamp((double)Qerr.W, -1.0, 1.0);
	const double angle = 2.0 * std::acos(w); // [0..pi]
	const double s = std::sqrt(FMath::Max(0.0, 1.0 - w*w));

	Eigen::Vector3d axis(1.0, 0.0, 0.0);
	if (s > 1e-9)
	{
		axis = Eigen::Vector3d(Qerr.X / s, Qerr.Y / s, Qerr.Z / s);
	}
	return axis * angle; // axis * angle (rad)
}

static void ComputeChainKinematics(
	const FTransform& BaseTransform,
	const TArray<float>& JointAnglesDeg,
	const TArray<FTransform>& JointLocalTransforms,
	const TArray<FVector>& JointAxesLocal,
	const FTransform& EndEffectorOffset,
	TArray<FVector>& OutJointPosWorld,
	TArray<FVector>& OutJointAxisWorld,
	FTransform& OutEEWorld)
{
	const int32 N = JointAnglesDeg.Num();
	OutJointPosWorld.SetNum(N);
	OutJointAxisWorld.SetNum(N);

	FTransform T = BaseTransform;

	for (int32 i = 0; i < N; i++)
	{
		// parent -> joint_i (q=0)
		T = JointLocalTransforms[i] * T;

		OutJointPosWorld[i] = T.GetLocation();

		// axis in joint frame -> world
		const FVector AxisWorld = SafeNormalAxis(T.TransformVectorNoScale(JointAxesLocal[i]));
		OutJointAxisWorld[i] = AxisWorld;

		// apply revolute rotation about axis (at joint origin)
		const float AngleRad = FMath::DegreesToRadians(JointAnglesDeg[i]);
		const FQuat R(AxisWorld, AngleRad);
		T.SetRotation((R * T.GetRotation()).GetNormalized());
	}

	OutEEWorld = EndEffectorOffset * T;
}

FTransform URammsIKLibrary::ComputeForwardKinematics(
	const FTransform& BaseTransform,
	const TArray<float>& JointAnglesDeg,
	const TArray<FTransform>& JointLocalTransforms,
	const TArray<FVector>& JointAxesLocal,
	const FTransform& EndEffectorOffset,
	bool bEnableDebugLogging)
{
	const int32 N = JointAnglesDeg.Num();
	if (JointLocalTransforms.Num() != N || JointAxesLocal.Num() != N)
	{
		UE_LOG(LogTemp, Warning, TEXT("[RammsIK] FK size mismatch (Angles=%d Local=%d Axes=%d)"),
			N, JointLocalTransforms.Num(), JointAxesLocal.Num());
		return BaseTransform;
	}

	TArray<FVector> JointPos, JointAxis;
	FTransform EE;
	ComputeChainKinematics(BaseTransform, JointAnglesDeg, JointLocalTransforms, JointAxesLocal, EndEffectorOffset, JointPos, JointAxis, EE);

	if (bEnableDebugLogging)
	{
		for (int32 i = 0; i < FMath::Min(N, 8); i++)
		{
			UE_LOG(LogTemp, Log, TEXT("[RammsIK] FK joint %d: pos=(%.2f,%.2f,%.2f) axis=(%.3f,%.3f,%.3f) q=%.2fdeg"),
				i, JointPos[i].X, JointPos[i].Y, JointPos[i].Z, JointAxis[i].X, JointAxis[i].Y, JointAxis[i].Z, JointAnglesDeg[i]);
		}
	}

	return EE;
}

FIKSolveResult URammsIKLibrary::SolveIK_FKChain(
	const FTransform& BaseTransform,
	const TArray<float>& CurrentAnglesDeg,
	const TArray<FTransform>& JointLocalTransforms,
	const TArray<FVector>& JointAxesLocal,
	const FTransform& EndEffectorOffset,
	const FTransform& TargetEndEffectorWorld,
	const TArray<FVector2D>& JointLimitsDeg,
	const TArray<bool>& TaskSpaceMask6,
	const TArray<float>& JointWeights,
	bool bEnableNullSpaceOptimization,
	float NullSpaceGain,
	const TArray<float>& NullSpaceBiasDeg,
	float Damping,
	float StepClipDeg,
	int32 MaxIterations,
	float PositionToleranceCm,
	float RotationToleranceDeg)
{
	FIKSolveResult Out;

	const int32 N = CurrentAnglesDeg.Num();
	if (N <= 0 ||
		JointLocalTransforms.Num() != N ||
		JointAxesLocal.Num() != N ||
		JointLimitsDeg.Num() != N)
	{
		Out.bSuccess = false;
		Out.JointAngles = CurrentAnglesDeg;
		return Out;
	}

	MaxIterations = FMath::Clamp(MaxIterations, 1, 500);

	// Mask vector m(6)
	double m[6] = {1,1,1,1,1,1};
	for (int i = 0; i < 6; i++)
	{
		if (TaskSpaceMask6.Num() == 6)
			m[i] = TaskSpaceMask6[i] ? 1.0 : 0.0;
	}

	// Joint weighting: higher means move less => use inverse as "allowance"
	Eigen::VectorXd Wdiag(N);
	for (int32 i = 0; i < N; i++)
	{
		double w = 1.0;
		if (JointWeights.Num() == N)
			w = FMath::Max(0.0f, JointWeights[i]);

		// If w==0 => freeze joint
		if (w <= 0.0)
			Wdiag(i) = 0.0;
		else
			Wdiag(i) = 1.0 / w;
	}
	const Eigen::MatrixXd W = Wdiag.asDiagonal();

	const double lambda = FMath::Max(0.0f, Damping);

	// Convert step clip
	const double stepClipRad = FMath::DegreesToRadians(FMath::Max(0.0f, StepClipDeg));

	// Balance units: treat rotation error as "cm-equivalent"
	const double RotToCm = 20.0;

	TArray<float> qDeg = CurrentAnglesDeg;

	TArray<FVector> JointPosWorld, JointAxisWorld;
	FTransform EEWorld = FTransform::Identity;

	double posErrCm = 0.0;
	double rotErrDeg = 0.0;
	
	static bool bLoggedBase = false;
	if (!bLoggedBase)
	{
		UE_LOG(LogTemp, Warning, TEXT("[IK Solver] BaseTransform: Loc=(%.1f, %.1f, %.1f) Rot=(%.1f, %.1f, %.1f)"),
			BaseTransform.GetLocation().X, BaseTransform.GetLocation().Y, BaseTransform.GetLocation().Z,
			BaseTransform.Rotator().Pitch, BaseTransform.Rotator().Yaw, BaseTransform.Rotator().Roll);
		UE_LOG(LogTemp, Warning, TEXT("[IK Solver] JointLocalTransforms[0]: Loc=(%.1f, %.1f, %.1f)"),
			JointLocalTransforms[0].GetLocation().X, JointLocalTransforms[0].GetLocation().Y, JointLocalTransforms[0].GetLocation().Z);
		bLoggedBase = true;
	}

	for (int32 it = 0; it < MaxIterations; it++)
	{
		ComputeChainKinematics(BaseTransform, qDeg, JointLocalTransforms, JointAxesLocal, EndEffectorOffset,
			JointPosWorld, JointAxisWorld, EEWorld);

		const Eigen::Vector3d pCur = ToEigen(EEWorld.GetLocation());
		const Eigen::Vector3d pTgt = ToEigen(TargetEndEffectorWorld.GetLocation());

		const Eigen::Vector3d dp = (pTgt - pCur); // cm
		const Eigen::Vector3d drot = RotationErrorAxisAngleRad(EEWorld.GetRotation().GetNormalized(),
			TargetEndEffectorWorld.GetRotation().GetNormalized()); // rad

		posErrCm = dp.norm();
		rotErrDeg = FMath::RadiansToDegrees((float)drot.norm());

		Out.PositionError = (float)posErrCm;
		Out.RotationError = (float)rotErrDeg;
		Out.IterationsUsed = it + 1;
		
		// Debug: Log FK vs target on first and last iteration
		static bool bLoggedOnce = false;
		if (!bLoggedOnce || it == MaxIterations - 1 || (posErrCm <= PositionToleranceCm && rotErrDeg <= RotationToleranceDeg))
		{
			UE_LOG(LogTemp, Warning, TEXT("[IK Solver] Iter %d: FK EE=(%.1f, %.1f, %.1f) Target=(%.1f, %.1f, %.1f) Error=%.1fcm"),
				it,
				EEWorld.GetLocation().X, EEWorld.GetLocation().Y, EEWorld.GetLocation().Z,
				TargetEndEffectorWorld.GetLocation().X, TargetEndEffectorWorld.GetLocation().Y, TargetEndEffectorWorld.GetLocation().Z,
				posErrCm);
			
			if (it == 0)
			{
				UE_LOG(LogTemp, Warning, TEXT("[IK Solver] Base: (%.1f, %.1f, %.1f)"),
					BaseTransform.GetLocation().X, BaseTransform.GetLocation().Y, BaseTransform.GetLocation().Z);
			}
			
			if (!bLoggedOnce) bLoggedOnce = true;
		}

		if (posErrCm <= PositionToleranceCm && rotErrDeg <= RotationToleranceDeg)
		{
			Out.bSuccess = true;
			break;
		}

		// Build error vector e (6x1), masked
		Eigen::Matrix<double, 6, 1> e;
		e.setZero();
		e(0) = m[0] * dp.x();
		e(1) = m[1] * dp.y();
		e(2) = m[2] * dp.z();

		// rotation vector components (world), scaled to cm
		e(3) = m[3] * (RotToCm * drot.x());
		e(4) = m[4] * (RotToCm * drot.y());
		e(5) = m[5] * (RotToCm * drot.z());

		// Jacobian J (6xN)
		Eigen::MatrixXd J(6, N);
		J.setZero();

		for (int32 j = 0; j < N; j++)
		{
			const FVector AxisU = JointAxisWorld[j].GetSafeNormal();
			if (AxisU.IsNearlyZero())
				continue;

			const Eigen::Vector3d pj = ToEigen(JointPosWorld[j]);
			const Eigen::Vector3d aj(AxisU.X, AxisU.Y, AxisU.Z);

			const Eigen::Vector3d Jv = aj.cross(pCur - pj); // cm / rad
			const Eigen::Vector3d Jw = aj;                  // rad / rad

			J(0, j) = m[0] * Jv.x();
			J(1, j) = m[1] * Jv.y();
			J(2, j) = m[2] * Jv.z();

			J(3, j) = m[3] * (RotToCm * Jw.x());
			J(4, j) = m[4] * (RotToCm * Jw.y());
			J(5, j) = m[5] * (RotToCm * Jw.z());
		}

		// DLS: dq = W J^T (J W J^T + λ^2 I)^-1 e
		const Eigen::MatrixXd JW = J * W; // 6xN
		Eigen::Matrix<double, 6, 6> A = (JW * J.transpose());
		A += (lambda * lambda) * Eigen::Matrix<double, 6, 6>::Identity();

		const Eigen::Matrix<double, 6, 1> x = A.ldlt().solve(e);
		Eigen::VectorXd dq = W * J.transpose() * x; // Nx1 (rad)

		// Null-space bias (optional)
		if (bEnableNullSpaceOptimization && NullSpaceGain > 0.0f && NullSpaceBiasDeg.Num() == N)
		{
			// J_pinv = W J^T A^-1
			const Eigen::Matrix<double, 6, 6> Ainv = A.ldlt().solve(Eigen::Matrix<double, 6, 6>::Identity());
			const Eigen::MatrixXd Jpinv = W * J.transpose() * Ainv; // Nx6

			Eigen::VectorXd qCurRad(N), qPrefRad(N);
			for (int32 i = 0; i < N; i++)
			{
				qCurRad(i) = FMath::DegreesToRadians(qDeg[i]);
				qPrefRad(i) = FMath::DegreesToRadians(NullSpaceBiasDeg[i]);
			}

			Eigen::VectorXd dqBias = (qPrefRad - qCurRad) * (double)NullSpaceGain;
			const Eigen::MatrixXd Nmat = (Eigen::MatrixXd::Identity(N, N) - (Jpinv * J));
			dq += Nmat * dqBias;
		}

		// Per-joint clip + apply limits
		for (int32 i = 0; i < N; i++)
		{
			double d = dq(i);

			if (stepClipRad > 0.0)
				d = FMath::Clamp((float)d, (float)-stepClipRad, (float)stepClipRad);

			double newDeg = qDeg[i] + FMath::RadiansToDegrees((float)d);

			const double minDeg = (double)JointLimitsDeg[i].X;
			const double maxDeg = (double)JointLimitsDeg[i].Y;
			if (minDeg < maxDeg)
				newDeg = FMath::Clamp((float)newDeg, (float)minDeg, (float)maxDeg);

			qDeg[i] = (float)newDeg;
		}
	}

	Out.JointAngles = qDeg;
	if (!Out.bSuccess)
	{
		Out.bSuccess = (posErrCm <= PositionToleranceCm) && (rotErrDeg <= RotationToleranceDeg);
	}
	return Out;
}

// Helper function to calculate signed angle between two vectors around an axis
static float CalculateSignedAngle(const FVector& From, const FVector& To, const FVector& Axis)
{
	// Project vectors onto plane perpendicular to axis
	FVector FromProjected = (From - Axis * FVector::DotProduct(From, Axis)).GetSafeNormal();
	FVector ToProjected = (To - Axis * FVector::DotProduct(To, Axis)).GetSafeNormal();

	// Calculate angle
	float CosAngle = FMath::Clamp(FVector::DotProduct(FromProjected, ToProjected), -1.0f, 1.0f);
	float Angle = FMath::Acos(CosAngle);

	// Determine sign using cross product
	FVector Cross = FVector::CrossProduct(FromProjected, ToProjected);
	float Sign = FVector::DotProduct(Cross, Axis);

	if (Sign < 0.0f)
	{
		Angle = -Angle;
	}

	return FMath::RadiansToDegrees(Angle);
}

// Helper function to apply hard joint limit constraints
static FVector ApplyJointLimitConstraint(
	const FVector& ParentPos,
	const FVector& JointPos,
	const FVector& ChildPos,
	const FVector& RefDirection,
	const FVector& AxisWorld,
	float MinAngleDeg,
	float MaxAngleDeg,
	float LinkLength)
{
	// Vector from joint to child (the direction we want to constrain)
	FVector JointToChild = (ChildPos - JointPos).GetSafeNormal();
	
	// Calculate current angle relative to reference direction
	// RefDirection is the direction the link points at angle=0 (from local transform)
	float CurrentAngle = CalculateSignedAngle(RefDirection, JointToChild, AxisWorld);

	// Hard clamp to limits
	float ClampedAngle = FMath::Clamp(CurrentAngle, MinAngleDeg, MaxAngleDeg);

	// If angle was clamped, reconstruct position
	if (!FMath::IsNearlyEqual(CurrentAngle, ClampedAngle, 0.01f))
	{
		// Rotate reference direction by clamped angle around axis
		float AngleRad = FMath::DegreesToRadians(ClampedAngle);
		FQuat Rotation(AxisWorld, AngleRad);
		FVector NewDirection = Rotation.RotateVector(RefDirection);
		
		return JointPos + NewDirection * LinkLength;
	}

	return ChildPos; // No clamping needed
}

FIKSolveResult URammsIKLibrary::SolveIK_FABRIK(
	const FTransform& BaseTransform,
	const TArray<float>& CurrentAnglesDeg,
	const TArray<FTransform>& JointLocalTransforms,
	const TArray<FVector>& JointAxesLocal,
	const TArray<FVector2D>& JointLimitsDeg,
	const FTransform& EndEffectorOffset,
	const FTransform& TargetEndEffectorWorld,
	const TArray<bool>& TaskSpaceMask6,
	int32 MaxIterations,
	float PositionToleranceCm)
{
	FIKSolveResult Result;
	Result.bSuccess = false;
	Result.IterationsUsed = 0;
	Result.PositionError = 0.0f;
	Result.RotationError = 0.0f;

	const int32 NumJoints = CurrentAnglesDeg.Num();
	if (NumJoints == 0 || NumJoints != JointLocalTransforms.Num() ||
		NumJoints != JointAxesLocal.Num() || NumJoints != JointLimitsDeg.Num())
	{
		UE_LOG(LogTemp, Warning, TEXT("[FABRIK] Invalid input: joint count mismatch"));
		return Result;
	}

	// Check if we should solve orientation (any of Roll/Pitch/Yaw enabled)
	bool bSolveOrientation = false;
	if (TaskSpaceMask6.Num() >= 6)
	{
		bSolveOrientation = TaskSpaceMask6[3] || TaskSpaceMask6[4] || TaskSpaceMask6[5];
	}

	// Build initial joint positions using FK from current angles
	TArray<FVector> JointPositions; // World space
	TArray<FVector> JointAxesWorld; // World space axes
	TArray<FVector> RefDirections;  // Reference directions (angle=0) for each link
	TArray<float> LinkLengths;
	
	JointPositions.SetNum(NumJoints + 1); // +1 for end-effector
	JointAxesWorld.SetNum(NumJoints);
	RefDirections.SetNum(NumJoints);
	LinkLengths.SetNum(NumJoints);

	// Compute forward kinematics to get initial positions
	FTransform CurrentTransform = BaseTransform;
	for (int32 i = 0; i < NumJoints; i++)
	{
		// CurrentTransform is currently in PARENT frame (or BASE for i=0)
		
		// Compute reference direction (link offset in parent frame -> world)
		// This is the direction the link points at angle=0
		FVector RefDirLocal = JointLocalTransforms[i].GetTranslation().GetSafeNormal();
		RefDirections[i] = CurrentTransform.TransformVectorNoScale(RefDirLocal).GetSafeNormal();
		
		// Apply local transform to get joint frame
		FTransform JointFrame = JointLocalTransforms[i] * CurrentTransform;

		// Store joint position (in world space)
		JointPositions[i] = JointFrame.GetLocation();
		
		// Compute world-space rotation axis
		// JointAxesLocal[i] is in JOINT's local frame, so transform by JointFrame
		JointAxesWorld[i] = JointFrame.TransformVectorNoScale(JointAxesLocal[i]).GetSafeNormal();

		// Apply current joint rotation
		float AngleRad = FMath::DegreesToRadians(CurrentAnglesDeg[i]);
		FQuat Rotation(JointAxesWorld[i], AngleRad);
		JointFrame.SetRotation(Rotation * JointFrame.GetRotation());
		
		// Update for next iteration
		CurrentTransform = JointFrame;
	}

	// Add end-effector position
	FTransform EETransform = EndEffectorOffset * CurrentTransform;
	JointPositions[NumJoints] = EETransform.GetLocation();

	// Compute link lengths
	for (int32 i = 0; i < NumJoints; i++)
	{
		LinkLengths[i] = FVector::Dist(JointPositions[i], JointPositions[i + 1]);
	}

	// Store base position (never moves)
	const FVector BasePosition = BaseTransform.GetLocation();
	const FVector TargetPosition = TargetEndEffectorWorld.GetLocation();

	// Check if target is reachable (sum of link lengths)
	float TotalReach = 0.0f;
	for (float Length : LinkLengths)
	{
		TotalReach += Length;
	}

	float DistanceToTarget = FVector::Dist(BasePosition, TargetPosition);
	if (DistanceToTarget > TotalReach)
	{
		// Target unreachable - stretch toward it
		FVector Direction = (TargetPosition - BasePosition).GetSafeNormal();
		FVector CurrentPos = BasePosition;
		for (int32 i = 0; i < NumJoints; i++)
		{
			JointPositions[i] = CurrentPos;
			CurrentPos += Direction * LinkLengths[i];
		}
		JointPositions[NumJoints] = CurrentPos;

		// Convert back to angles and return
		Result.JointAngles = CurrentAnglesDeg; // Keep original angles
		Result.PositionError = DistanceToTarget - TotalReach;
		Result.bSuccess = false;
		return Result;
	}

	// FABRIK iteration
	for (int32 Iteration = 0; Iteration < MaxIterations; Iteration++)
	{
		Result.IterationsUsed = Iteration + 1;

		// Check convergence
		float CurrentError = FVector::Dist(JointPositions[NumJoints], TargetPosition);
		if (CurrentError < PositionToleranceCm)
		{
			Result.bSuccess = true;
			Result.PositionError = CurrentError;
			break;
		}

		// ===== BACKWARD PASS: Start from target, reach back to base =====
		JointPositions[NumJoints] = TargetPosition;

		for (int32 i = NumJoints - 1; i >= 0; i--)
		{
			// Direction from next joint to current joint
			FVector Direction = (JointPositions[i] - JointPositions[i + 1]).GetSafeNormal();
			
			// New position maintaining link length
			FVector NewPosition = JointPositions[i + 1] + Direction * LinkLengths[i];

			// Apply joint limit constraints for this joint
			// Joint i connects JointPositions[i] to JointPositions[i+1]
			if (i < NumJoints) // Valid joint
			{
				NewPosition = ApplyJointLimitConstraint(
					(i > 0) ? JointPositions[i - 1] : BasePosition, // Parent position
					JointPositions[i],     // Current joint position (will be NewPosition after)
					JointPositions[i + 1], // Child position (end of link)
					RefDirections[i],      // Reference direction for this link
					JointAxesWorld[i],     // Rotation axis for this joint
					JointLimitsDeg[i].X,   // Min angle
					JointLimitsDeg[i].Y,   // Max angle
					LinkLengths[i]);
			}

			JointPositions[i] = NewPosition;
		}

		// ===== FORWARD PASS: Start from base, reach toward target =====
		JointPositions[0] = BasePosition;

		for (int32 i = 0; i < NumJoints; i++)
		{
			// Direction from current joint to next joint
			FVector Direction = (JointPositions[i + 1] - JointPositions[i]).GetSafeNormal();
			
			// New position maintaining link length
			FVector NewPosition = JointPositions[i] + Direction * LinkLengths[i];

			// Apply joint limit constraints
			if (i < NumJoints) // Has a child
			{
				NewPosition = ApplyJointLimitConstraint(
					(i > 0) ? JointPositions[i - 1] : BasePosition, // Parent position
					JointPositions[i],     // Current joint position
					NewPosition,           // Proposed next position
					RefDirections[i],      // Reference direction for this link
					JointAxesWorld[i],     // Rotation axis
					JointLimitsDeg[i].X,   // Min angle
					JointLimitsDeg[i].Y,   // Max angle
					LinkLengths[i]);
			}

			JointPositions[i + 1] = NewPosition;
		}
	}

	// Convert final positions back to joint angles
	TArray<float> SolvedAngles;
	SolvedAngles.SetNum(NumJoints);

	FTransform FK_Transform = BaseTransform;
	for (int32 i = 0; i < NumJoints; i++)
	{
		// Get direction from parent to next joint in FABRIK solution
		FVector FABRIKDirection = (JointPositions[i + 1] - JointPositions[i]).GetSafeNormal();

		// Use pre-computed reference direction and axis (already in world space)
		// These were computed during FK initialization and stay constant during FABRIK
		float Angle = CalculateSignedAngle(RefDirections[i], FABRIKDirection, JointAxesWorld[i]);
		
		// Clamp to joint limits
		Angle = FMath::Clamp(Angle, JointLimitsDeg[i].X, JointLimitsDeg[i].Y);
		SolvedAngles[i] = Angle;
	}

	Result.JointAngles = SolvedAngles;
	Result.PositionError = FVector::Dist(JointPositions[NumJoints], TargetPosition);

	if (!Result.bSuccess)
	{
		Result.bSuccess = (Result.PositionError <= PositionToleranceCm);
	}

	return Result;
}
