#include "RammsIKLibrary.h"

#include "FABRIK.h"
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

static bool IsFreeLimit(const FVector2D& LimitsDeg)
{
	const float Range = LimitsDeg.Y - LimitsDeg.X;
	return Range >= 359.0f;
}

static float WrapAngleDeg(float AngleDeg)
{
	return FMath::UnwindDegrees(AngleDeg);
}

static float ApplyAngleLimits(float AngleDeg, const FVector2D& LimitsDeg, float LimitEscapeDeg)
{
	if (IsFreeLimit(LimitsDeg))
	{
		return WrapAngleDeg(AngleDeg);
	}

	const float Clamped = FMath::Clamp(AngleDeg, LimitsDeg.X, LimitsDeg.Y);
	if (FMath::IsNearlyEqual(Clamped, AngleDeg, 1e-3f) || LimitEscapeDeg <= 0.0f)
	{
		return Clamped;
	}

	// Nudge inward if we hit a hard limit
	const float Center = 0.5f * (LimitsDeg.X + LimitsDeg.Y);
	const float Dir = (Clamped < Center) ? 1.0f : -1.0f;
	return FMath::Clamp(Clamped + Dir * LimitEscapeDeg, LimitsDeg.X, LimitsDeg.Y);
}

FIKSolveResult URammsIKLibrary::SolveIK_CCD(
	const FTransform& BaseTransform,
	const TArray<float>& CurrentAnglesDeg,
	const TArray<FTransform>& JointLocalTransforms,
	const TArray<FVector>& JointAxesLocal,
	const TArray<FVector2D>& JointLimitsDeg,
	const FTransform& EndEffectorOffset,
	const FTransform& TargetEndEffectorWorld,
	const TArray<bool>& TaskSpaceMask6,
	int32 MaxIterations,
	float PositionToleranceCm,
	float RotationToleranceDeg,
	float PositionGain,
	float OrientationGain,
	float MaxAngleStepDeg)
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
		UE_LOG(LogTemp, Warning, TEXT("[CCD] Invalid input: joint count mismatch"));
		return Result;
	}

	const FVector TargetPosition = TargetEndEffectorWorld.GetLocation();
	TArray<float> Angles = CurrentAnglesDeg;
	TArray<FVector> JointPosWorld;
	TArray<FVector> JointAxisWorld;
	FTransform EEWorld;

	bool bSolveOrientation = false;
	if (TaskSpaceMask6.Num() >= 6)
	{
		bSolveOrientation = TaskSpaceMask6[3] || TaskSpaceMask6[4] || TaskSpaceMask6[5];
	}

	for (int32 Iteration = 0; Iteration < MaxIterations; Iteration++)
	{
		Result.IterationsUsed = Iteration + 1;

		ComputeChainKinematics(
			BaseTransform,
			Angles,
			JointLocalTransforms,
			JointAxesLocal,
			EndEffectorOffset,
			JointPosWorld,
			JointAxisWorld,
			EEWorld);

		const float PosErr = FVector::Dist(EEWorld.GetLocation(), TargetPosition);
		const FQuat RotErrQ = TargetEndEffectorWorld.GetRotation() * EEWorld.GetRotation().Inverse();
		const float RotErrDeg = FMath::RadiansToDegrees(RotErrQ.GetAngle());

		if (!bSolveOrientation)
		{
			if (PosErr <= PositionToleranceCm)
			{
				Result.bSuccess = true;
				Result.PositionError = PosErr;
				Result.RotationError = RotErrDeg;
				break;
			}
		}
		else if (PosErr <= PositionToleranceCm && RotErrDeg <= RotationToleranceDeg)
		{
			Result.bSuccess = true;
			Result.PositionError = PosErr;
			Result.RotationError = RotErrDeg;
			break;
		}

		for (int32 i = NumJoints - 1; i >= 0; i--)
		{
			ComputeChainKinematics(
				BaseTransform,
				Angles,
				JointLocalTransforms,
				JointAxesLocal,
				EndEffectorOffset,
				JointPosWorld,
				JointAxisWorld,
				EEWorld);

			const FVector JointPos = JointPosWorld[i];
			const FVector Axis = (i < JointAxisWorld.Num()) ? JointAxisWorld[i] : FVector::XAxisVector;

			const FVector ToEE = (EEWorld.GetLocation() - JointPos).GetSafeNormal();
			const FVector ToTarget = (TargetPosition - JointPos).GetSafeNormal();

			float DeltaPos = CalculateSignedAngle(ToEE, ToTarget, Axis) * PositionGain;

			float DeltaOri = 0.0f;
			if (bSolveOrientation && OrientationGain > 0.0f)
			{
				const FQuat Qerr = TargetEndEffectorWorld.GetRotation() * EEWorld.GetRotation().Inverse();
				float AngleRad;
				FVector ErrAxis;
				Qerr.ToAxisAndAngle(ErrAxis, AngleRad);
				const float Sign = FVector::DotProduct(ErrAxis.GetSafeNormal(), Axis.GetSafeNormal()) >= 0.0f ? 1.0f : -1.0f;
				DeltaOri = FMath::RadiansToDegrees(AngleRad) * Sign * OrientationGain;
			}

			float Delta = DeltaPos + DeltaOri;
			Delta = FMath::Clamp(Delta, -MaxAngleStepDeg, MaxAngleStepDeg);

			float NewAngle = Angles[i] + Delta;
			if (IsFreeLimit(JointLimitsDeg[i]))
			{
				NewAngle = WrapAngleDeg(NewAngle);
			}
			else
			{
				NewAngle = FMath::Clamp(NewAngle, JointLimitsDeg[i].X, JointLimitsDeg[i].Y);
			}
			Angles[i] = NewAngle;
		}
	}

	ComputeChainKinematics(
		BaseTransform,
		Angles,
		JointLocalTransforms,
		JointAxesLocal,
		EndEffectorOffset,
		JointPosWorld,
		JointAxisWorld,
		EEWorld);

	Result.JointAngles = Angles;
	Result.PositionError = FVector::Dist(EEWorld.GetLocation(), TargetPosition);
	const FQuat FinalRotErr = TargetEndEffectorWorld.GetRotation() * EEWorld.GetRotation().Inverse();
	Result.RotationError = FMath::RadiansToDegrees(FinalRotErr.GetAngle());

	if (!Result.bSuccess)
	{
		Result.bSuccess = (Result.PositionError <= PositionToleranceCm);
		if (bSolveOrientation)
		{
			Result.bSuccess = Result.bSuccess && (Result.RotationError <= RotationToleranceDeg);
		}
	}

	return Result;
}

FIKSolveResult URammsIKLibrary::SolveIK_UEFabrik(
	const FTransform& BaseTransform,
	const TArray<float>& CurrentAnglesDeg,
	const TArray<FTransform>& JointLocalTransforms,
	const TArray<FVector>& JointAxesLocal,
	const TArray<FVector2D>& JointLimitsDeg,
	const FTransform& EndEffectorOffset,
	const FTransform& TargetEndEffectorWorld,
	const TArray<bool>& TaskSpaceMask6,
	int32 MaxIterations,
	float PositionToleranceCm,
	float RotationToleranceDeg,
	float LimitEscapeDeg,
	bool bAxesInParentFrame)
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
		UE_LOG(LogTemp, Warning, TEXT("[UEFABRIK] Invalid input: joint count mismatch"));
		return Result;
	}

	TArray<FVector> JointPosWorld;
	TArray<FVector> JointAxisWorld;
	FTransform EEWorld;
	ComputeChainKinematics(
		BaseTransform,
		CurrentAnglesDeg,
		JointLocalTransforms,
		JointAxesLocal,
		EndEffectorOffset,
		JointPosWorld,
		JointAxisWorld,
		EEWorld);

	TArray<FVector> ChainPositions;
	ChainPositions.SetNum(NumJoints + 1);
	for (int32 i = 0; i < NumJoints; i++)
	{
		ChainPositions[i] = JointPosWorld[i];
	}
	ChainPositions[NumJoints] = EEWorld.GetLocation();

	TArray<double> LinkLengths;
	LinkLengths.SetNum(NumJoints + 1);
	LinkLengths[0] = 0.0;
	double MaximumReach = 0.0;
	for (int32 i = 1; i < NumJoints + 1; i++)
	{
		const double Len = FVector::Dist(ChainPositions[i], ChainPositions[i - 1]);
		LinkLengths[i] = Len;
		MaximumReach += Len;
	}

	TArray<FFABRIKChainLink> Chain;
	Chain.Reserve(NumJoints + 1);
	for (int32 i = 0; i < NumJoints + 1; i++)
	{
		Chain.Add(FFABRIKChainLink(ChainPositions[i], LinkLengths[i], i, i));
	}

	MaxIterations = FMath::Clamp(MaxIterations, 1, 500);
	const bool bModified = AnimationCore::SolveFabrik(
		Chain,
		TargetEndEffectorWorld.GetLocation(),
		MaximumReach,
		PositionToleranceCm,
		MaxIterations);
	Result.IterationsUsed = MaxIterations;

	// Reconstruct joint angles from solved positions using hinge-projection in joint frames
	TArray<float> Angles = CurrentAnglesDeg;
	FTransform ParentTransform = BaseTransform;

	for (int32 i = 0; i < NumJoints; i++)
	{
		FTransform JointFrame = JointLocalTransforms[i] * ParentTransform; // q=0 frame

		const FVector CurrentChildWorld = (i + 1 < NumJoints) ? JointPosWorld[i + 1] : EEWorld.GetLocation();
		const FVector DesiredChildWorld = Chain[i + 1].Position;
		const FVector DesiredChildJoint = JointFrame.InverseTransformPositionNoScale(DesiredChildWorld);
		FVector DesiredDirJoint = DesiredChildJoint.GetSafeNormal();

		const FVector CurrentChildJoint = JointFrame.InverseTransformPositionNoScale(CurrentChildWorld);
		FVector RefDirJoint = CurrentChildJoint.GetSafeNormal();
		if (!RefDirJoint.Normalize())
		{
			RefDirJoint = FVector::XAxisVector;
		}

		FVector AxisJoint = JointAxesLocal[i].GetSafeNormal();
		if (bAxesInParentFrame)
		{
			// Convert axis from parent frame into joint frame
			AxisJoint = JointLocalTransforms[i].GetRotation().UnrotateVector(AxisJoint).GetSafeNormal();
		}
		if (DesiredDirJoint.IsNearlyZero())
		{
			DesiredDirJoint = RefDirJoint;
		}

		const float TargetAngle = CalculateSignedAngle(RefDirJoint, DesiredDirJoint, AxisJoint);
		float NewAngle = ApplyAngleLimits(TargetAngle, JointLimitsDeg[i], LimitEscapeDeg);
		Angles[i] = NewAngle;

		const FVector AxisWorld = JointFrame.TransformVectorNoScale(AxisJoint).GetSafeNormal();
		JointFrame.SetRotation((FQuat(AxisWorld, FMath::DegreesToRadians(NewAngle)) * JointFrame.GetRotation()).GetNormalized());
		ParentTransform = JointFrame;
	}

	ComputeChainKinematics(
		BaseTransform,
		Angles,
		JointLocalTransforms,
		JointAxesLocal,
		EndEffectorOffset,
		JointPosWorld,
		JointAxisWorld,
		EEWorld);

	Result.JointAngles = Angles;
	Result.PositionError = FVector::Dist(EEWorld.GetLocation(), TargetEndEffectorWorld.GetLocation());
	const FQuat FinalRotErr = TargetEndEffectorWorld.GetRotation() * EEWorld.GetRotation().Inverse();
	Result.RotationError = FMath::RadiansToDegrees(FinalRotErr.GetAngle());

	bool bSolveOrientation = false;
	if (TaskSpaceMask6.Num() >= 6)
	{
		bSolveOrientation = TaskSpaceMask6[3] || TaskSpaceMask6[4] || TaskSpaceMask6[5];
	}

	Result.bSuccess = (Result.PositionError <= PositionToleranceCm);
	if (bSolveOrientation)
	{
		Result.bSuccess = Result.bSuccess && (Result.RotationError <= RotationToleranceDeg);
	}

	if (!bModified)
	{
		// Solver reported no modifications; still return current angles and error metrics.
		Result.JointAngles = Angles;
	}

	return Result;
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
	float PositionToleranceCm,
	float RotationToleranceDeg,
	float AngleGain,
	float MaxAngleStepDeg,
	float LimitEscapeDeg,
	int32 OrientationIterations,
	float OrientationGain)
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

	// Axis-constrained FABRIK implemented as angle-space solver:
	// Iterate with joint axes derived from the current chain and rotate each joint
	// to align the end-effector direction with the target.
	TArray<float> Angles = CurrentAnglesDeg;
	const FVector TargetPosition = TargetEndEffectorWorld.GetLocation();

	TArray<FVector> JointPosWorld;
	TArray<FVector> JointAxisWorld;
	FTransform EEWorld;

	// Optional orientation refinement using the last few joints.
	bool bSolveOrientation = false;
	if (TaskSpaceMask6.Num() >= 6)
	{
		bSolveOrientation = TaskSpaceMask6[3] || TaskSpaceMask6[4] || TaskSpaceMask6[5];
	}

	for (int32 Iteration = 0; Iteration < MaxIterations; Iteration++)
	{
		Result.IterationsUsed = Iteration + 1;

		ComputeChainKinematics(
			BaseTransform,
			Angles,
			JointLocalTransforms,
			JointAxesLocal,
			EndEffectorOffset,
			JointPosWorld,
			JointAxisWorld,
			EEWorld);

		const float CurrentError = FVector::Dist(EEWorld.GetLocation(), TargetPosition);
		const float GainScale = FMath::Clamp(CurrentError / FMath::Max(PositionToleranceCm * 5.0f, 1.0f), 0.1f, 1.0f);
		if (!bSolveOrientation)
		{
			if (CurrentError <= PositionToleranceCm)
			{
				Result.bSuccess = true;
				Result.PositionError = CurrentError;
				break;
			}
		}
		else
		{
			const FQuat CurRotErr = TargetEndEffectorWorld.GetRotation() * EEWorld.GetRotation().Inverse();
			const float CurRotDeg = FMath::RadiansToDegrees(CurRotErr.GetAngle());
			if (CurrentError <= PositionToleranceCm && CurRotDeg <= RotationToleranceDeg)
			{
				Result.bSuccess = true;
				Result.PositionError = CurrentError;
				Result.RotationError = CurRotDeg;
				break;
			}
		}

		// Stage 1: Hinge-projected FABRIK (backward pass in joint angle space)
		FVector DesiredChildPos = TargetPosition;
		for (int32 i = NumJoints - 1; i >= 0; i--)
		{
			const FVector JointPos = JointPosWorld[i];
			const FVector DesiredDirWorld = (DesiredChildPos - JointPos).GetSafeNormal();

			// Parent frame rotation (from base to parent of this joint)
			FTransform ParentTransform = BaseTransform;
			for (int32 k = 0; k < i; k++)
			{
				const FVector AxisWorld = JointAxisWorld[k];
				const float AngleRad = FMath::DegreesToRadians(Angles[k]);
				FTransform JointFrame = JointLocalTransforms[k] * ParentTransform;
				JointFrame.SetRotation((FQuat(AxisWorld, AngleRad) * JointFrame.GetRotation()).GetNormalized());
				ParentTransform = JointFrame;
			}

			// Convert desired direction to parent frame
			const FVector DesiredDirParent = ParentTransform.InverseTransformVectorNoScale(DesiredDirWorld).GetSafeNormal();
			const FVector AxisParent = JointLocalTransforms[i].TransformVectorNoScale(JointAxesLocal[i]).GetSafeNormal();
			const FVector RefDirParent = JointLocalTransforms[i].GetTranslation().GetSafeNormal();

			// Project desired direction onto hinge plane
			FVector DesiredProj = DesiredDirParent - AxisParent * FVector::DotProduct(DesiredDirParent, AxisParent);
			if (!DesiredProj.Normalize())
			{
				continue;
			}

			float TargetAngle = CalculateSignedAngle(RefDirParent, DesiredProj, AxisParent);
			float Delta = (TargetAngle - Angles[i]) * AngleGain * GainScale;
			Delta = FMath::Clamp(Delta, -MaxAngleStepDeg, MaxAngleStepDeg);

			float NewAngle = Angles[i] + Delta;
			NewAngle = ApplyAngleLimits(NewAngle, JointLimitsDeg[i], LimitEscapeDeg);
			Angles[i] = NewAngle;

			// Update desired child for next joint up the chain (keep link length)
			const float LinkLength = JointLocalTransforms[i].GetTranslation().Size();
			const FVector NewDirParent = FQuat(AxisParent, FMath::DegreesToRadians(NewAngle)).RotateVector(RefDirParent);
			const FVector NewDirWorld = ParentTransform.TransformVectorNoScale(NewDirParent).GetSafeNormal();
			DesiredChildPos = JointPos - NewDirWorld * LinkLength;
		}

		// Recompute chain after backward pass
		ComputeChainKinematics(
			BaseTransform,
			Angles,
			JointLocalTransforms,
			JointAxesLocal,
			EndEffectorOffset,
			JointPosWorld,
			JointAxisWorld,
			EEWorld);
	}

	if (bSolveOrientation)
	{
		// Small CCD-style orientation correction using end-effector rotation error.
		const int32 StartIdx = FMath::Max(0, NumJoints - 3);
		for (int32 Iter = 0; Iter < OrientationIterations; Iter++)
		{
			ComputeChainKinematics(
				BaseTransform,
				Angles,
				JointLocalTransforms,
				JointAxesLocal,
				EndEffectorOffset,
				JointPosWorld,
				JointAxisWorld,
				EEWorld);

			const FQuat Qerr = TargetEndEffectorWorld.GetRotation() * EEWorld.GetRotation().Inverse();
			if (Qerr.IsIdentity())
			{
				break;
			}

			for (int32 i = NumJoints - 1; i >= StartIdx; i--)
			{
				const FVector Axis = (i < JointAxisWorld.Num()) ? JointAxisWorld[i] : FVector::XAxisVector;
				// Rotate around joint axis to reduce orientation error
				float AngleRad;
				FVector ErrAxis;
				Qerr.ToAxisAndAngle(ErrAxis, AngleRad);

				const float Sign = FVector::DotProduct(ErrAxis.GetSafeNormal(), Axis.GetSafeNormal()) >= 0.0f ? 1.0f : -1.0f;
				float DeltaDeg = FMath::RadiansToDegrees(AngleRad) * Sign * OrientationGain;
				DeltaDeg = FMath::Clamp(DeltaDeg, -MaxAngleStepDeg * OrientationGain, MaxAngleStepDeg * OrientationGain);

				float NewAngle = Angles[i] + DeltaDeg;
				NewAngle = ApplyAngleLimits(NewAngle, JointLimitsDeg[i], LimitEscapeDeg);
				Angles[i] = NewAngle;
			}
		}
	}

	Result.JointAngles = Angles;
	// Final error metrics
	Result.PositionError = FVector::Dist(EEWorld.GetLocation(), TargetPosition);
	const FQuat FinalRotErr = TargetEndEffectorWorld.GetRotation() * EEWorld.GetRotation().Inverse();
	Result.RotationError = FMath::RadiansToDegrees(FinalRotErr.GetAngle());

	if (!Result.bSuccess)
	{
		Result.bSuccess = (Result.PositionError <= PositionToleranceCm);
		if (bSolveOrientation)
		{
			Result.bSuccess = Result.bSuccess && (Result.RotationError <= RotationToleranceDeg);
		}
	}

	return Result;
}
