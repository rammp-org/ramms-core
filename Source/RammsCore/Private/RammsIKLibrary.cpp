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

		// DLS: dq = W J^T (J W J^T + \u03bb^2 I)^-1 e
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

// Helper function to calculate signed angle between two vectors around an axis.
// Returns 0 if either vector is (nearly) parallel to the axis (degenerate hinge projection).
static float CalculateSignedAngle(const FVector& From, const FVector& To, const FVector& Axis)
{
	// Project vectors onto plane perpendicular to axis
	FVector FromRaw = From - Axis * FVector::DotProduct(From, Axis);
	FVector ToRaw = To - Axis * FVector::DotProduct(To, Axis);

	// Guard against degenerate projections (vector nearly parallel to axis)
	const float FromLen = FromRaw.Size();
	const float ToLen = ToRaw.Size();
	if (FromLen < KINDA_SMALL_NUMBER || ToLen < KINDA_SMALL_NUMBER)
	{
		return 0.0f;
	}

	FVector FromProjected = FromRaw / FromLen;
	FVector ToProjected = ToRaw / ToLen;

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

// Ensure quaternion is in shortest-path form (W >= 0)
static FQuat ShortestPath(const FQuat& Q)
{
	FQuat R = Q;
	R.Normalize();
	if (R.W < 0.0f) { R.X = -R.X; R.Y = -R.Y; R.Z = -R.Z; R.W = -R.W; }
	return R;
}

// Compute rotation error in degrees (shortest path)
static float RotationErrorDeg(const FQuat& Current, const FQuat& Target)
{
	FQuat Err = ShortestPath(Target * Current.Inverse());
	return FMath::RadiansToDegrees(Err.GetAngle());
}

// =============================================================================
// CCD Solver (Cyclic Coordinate Descent, hinge-constrained)
//
// Uses the SAME FK (ComputeChainKinematics) and helpers as DLS and FABRIK.
// Classic CCD: re-FK per joint in the sweep for exact (non-linearized) geometry.
// For each joint, computes the exact hinge-plane angle between projected
// (joint->EE) and (joint->target) vectors.
//
// When orientation is enabled, position and orientation deltas are blended
// per-joint using the respective gains.
// =============================================================================
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

	const int32 N = CurrentAnglesDeg.Num();
	if (N == 0 || N != JointLocalTransforms.Num() ||
		N != JointAxesLocal.Num() || N != JointLimitsDeg.Num())
	{
		UE_LOG(LogTemp, Warning, TEXT("[CCD] Invalid input: joint count mismatch"));
		return Result;
	}

	const FVector TargetPos = TargetEndEffectorWorld.GetLocation();
	MaxIterations = FMath::Clamp(MaxIterations, 1, 500);

	bool bSolveOrientation = false;
	if (TaskSpaceMask6.Num() >= 6)
	{
		bSolveOrientation = TaskSpaceMask6[3] || TaskSpaceMask6[4] || TaskSpaceMask6[5];
	}

	TArray<float> Angles = CurrentAnglesDeg;
	TArray<FVector> JointPosWorld;
	TArray<FVector> JointAxisWorld;
	FTransform EEWorld;

	// No LimitEscape for CCD — use 0
	const float LimitEscapeDeg = 0.0f;

	for (int32 Iter = 0; Iter < MaxIterations; Iter++)
	{
		Result.IterationsUsed = Iter + 1;

		// FK for convergence check
		ComputeChainKinematics(BaseTransform, Angles, JointLocalTransforms,
			JointAxesLocal, EndEffectorOffset, JointPosWorld, JointAxisWorld, EEWorld);

		const float PosErr = FVector::Dist(EEWorld.GetLocation(), TargetPos);
		const float RotErr = RotationErrorDeg(EEWorld.GetRotation(), TargetEndEffectorWorld.GetRotation());
		Result.PositionError = PosErr;
		Result.RotationError = RotErr;

		if (PosErr <= PositionToleranceCm && (!bSolveOrientation || RotErr <= RotationToleranceDeg))
		{
			Result.bSuccess = true;
			break;
		}

		// === CCD sweep: tip to root ===
		// Re-FK per joint for exact geometry (the key CCD vs FABRIK difference)
		for (int32 i = N - 1; i >= 0; i--)
		{
			// Recompute FK with current angles (reflects prior joint updates this sweep)
			ComputeChainKinematics(BaseTransform, Angles, JointLocalTransforms,
				JointAxesLocal, EndEffectorOffset, JointPosWorld, JointAxisWorld, EEWorld);

			const FVector JointPos = JointPosWorld[i];
			const FVector AxisW = JointAxisWorld[i].GetSafeNormal();
			const FVector EEPos = EEWorld.GetLocation();

			if (AxisW.IsNearlyZero())
				continue;

			// --- Position component: exact hinge-plane angle ---
			float PosDeltaDeg = 0.0f;
			{
				const FVector ToEE = EEPos - JointPos;
				const FVector ToTarget = TargetPos - JointPos;

				// Project onto hinge plane (perpendicular to joint axis)
				const FVector ToEEProj = ToEE - AxisW * FVector::DotProduct(ToEE, AxisW);
				const FVector ToTargetProj = ToTarget - AxisW * FVector::DotProduct(ToTarget, AxisW);

				if (ToEEProj.SizeSquared() > KINDA_SMALL_NUMBER &&
					ToTargetProj.SizeSquared() > KINDA_SMALL_NUMBER)
				{
					const FVector FromN = ToEEProj.GetSafeNormal();
					const FVector ToN = ToTargetProj.GetSafeNormal();
					const float CosA = FMath::Clamp(FVector::DotProduct(FromN, ToN), -1.0f, 1.0f);
					PosDeltaDeg = FMath::RadiansToDegrees(FMath::Acos(CosA));

					// Sign via right-hand rule around axis
					if (FVector::DotProduct(FVector::CrossProduct(FromN, ToN), AxisW) < 0.0f)
					{
						PosDeltaDeg = -PosDeltaDeg;
					}
				}
			}

			// --- Orientation component: project rotation error onto joint axis ---
			float OriDeltaDeg = 0.0f;
			if (bSolveOrientation && OrientationGain > 0.0f)
			{
				const Eigen::Vector3d drot = RotationErrorAxisAngleRad(
					EEWorld.GetRotation().GetNormalized(),
					TargetEndEffectorWorld.GetRotation().GetNormalized());

				const FVector dr((float)drot.x(), (float)drot.y(), (float)drot.z());
				// Project rotation error onto this joint's axis
				const float JwDotDr = FVector::DotProduct(AxisW, dr); // rad
				OriDeltaDeg = FMath::RadiansToDegrees(JwDotDr);
			}

			// --- Blend position and orientation ---
			float DeltaDeg = PositionGain * PosDeltaDeg + OrientationGain * OriDeltaDeg;

			// Step clamp
			DeltaDeg = FMath::Clamp(DeltaDeg, -MaxAngleStepDeg, MaxAngleStepDeg);

			float NewAngle = Angles[i] + DeltaDeg;
			NewAngle = ApplyAngleLimits(NewAngle, JointLimitsDeg[i], LimitEscapeDeg);
			Angles[i] = NewAngle;
		}
	}

	// Final FK for error metrics
	ComputeChainKinematics(BaseTransform, Angles, JointLocalTransforms,
		JointAxesLocal, EndEffectorOffset, JointPosWorld, JointAxisWorld, EEWorld);

	Result.JointAngles = Angles;
	Result.PositionError = FVector::Dist(EEWorld.GetLocation(), TargetPos);
	Result.RotationError = RotationErrorDeg(EEWorld.GetRotation(), TargetEndEffectorWorld.GetRotation());

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
	// Stub: UE-FABRIK removed.
	FIKSolveResult Result;
	Result.bSuccess = false;
	Result.JointAngles = CurrentAnglesDeg;
	Result.PositionError = 999.0f;
	Result.RotationError = 999.0f;
	Result.IterationsUsed = 0;
	UE_LOG(LogTemp, Warning, TEXT("[UEFabrik] Solver removed. Use DLS or FABRIK."));
	return Result;
}

// =============================================================================
// FABRIK Solver (angle-space, hinge-constrained)
//
// Operates directly in joint-angle space using the same FK as DLS.
// Each iteration sweeps from tip to root. For each joint:
//   1. Compute FK to get current EE position and joint axis/position
//   2. Find the signed angle (on the hinge plane perpendicular to the axis)
//      between (joint->EE) and (joint->target)
//   3. Apply gain, step clamp, and joint limits
//   4. Update the angle and move to the next joint
//
// This is geometrically equivalent to CCD with 1-DOF hinge constraints,
// but avoids all position-space / angle-space conversion issues.
// Twist joints (axis parallel to EE direction) naturally get zero delta
// because their hinge-plane projection is degenerate.
// =============================================================================
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

	const int32 N = CurrentAnglesDeg.Num();
	if (N == 0 || N != JointLocalTransforms.Num() ||
		N != JointAxesLocal.Num() || N != JointLimitsDeg.Num())
	{
		UE_LOG(LogTemp, Warning, TEXT("[FABRIK] Invalid input: joint count mismatch"));
		return Result;
	}

	const FVector TargetPos = TargetEndEffectorWorld.GetLocation();
	MaxIterations = FMath::Clamp(MaxIterations, 1, 500);

	bool bSolveOrientation = false;
	if (TaskSpaceMask6.Num() >= 6)
	{
		bSolveOrientation = TaskSpaceMask6[3] || TaskSpaceMask6[4] || TaskSpaceMask6[5];
	}

	TArray<float> Angles = CurrentAnglesDeg;
	TArray<FVector> JointPosWorld;
	TArray<FVector> JointAxisWorld;
	FTransform EEWorld;

	// Damping for per-joint scalar DLS (prevents huge steps when Jv is small)
	const double Lambda2 = 0.01; // cm^2 damping (small — we rely on step clamp for stability)
	const double StepClipRad = FMath::DegreesToRadians(MaxAngleStepDeg);

	for (int32 Iter = 0; Iter < MaxIterations; Iter++)
	{
		Result.IterationsUsed = Iter + 1;

		// === Single FK evaluation per iteration (same as DLS) ===
		ComputeChainKinematics(BaseTransform, Angles, JointLocalTransforms,
			JointAxesLocal, EndEffectorOffset, JointPosWorld, JointAxisWorld, EEWorld);

		const FVector EEPos = EEWorld.GetLocation();
		const FVector dp = TargetPos - EEPos; // position error (cm)

		// Check convergence
		const float PosErr = (float)dp.Size();
		const float RotErr = RotationErrorDeg(EEWorld.GetRotation(), TargetEndEffectorWorld.GetRotation());
		Result.PositionError = PosErr;
		Result.RotationError = RotErr;

		if (PosErr <= PositionToleranceCm && (!bSolveOrientation || RotErr <= RotationToleranceDeg))
		{
			Result.bSuccess = true;
			break;
		}

		// === Compute all Jacobian columns from this ONE FK snapshot ===
		// Then solve each joint sequentially, tracking the residual error.
		// As each joint is updated, we subtract its predicted contribution
		// from the remaining error (like Gauss-Seidel on the Jacobian).
		FVector dpResidual = dp;

		for (int32 i = N - 1; i >= 0; i--)
		{
			const FVector AxisW = JointAxisWorld[i].GetSafeNormal();
			if (AxisW.IsNearlyZero())
				continue;

			// Jacobian velocity column: Jv = axis x (EE - joint)
			// Uses the ORIGINAL EE position from this iteration's FK,
			// same as how DLS builds its full Jacobian.
			const FVector Jv = FVector::CrossProduct(AxisW, EEPos - JointPosWorld[i]); // cm/rad

			const double JvDotDp = FVector::DotProduct(Jv, dpResidual);
			const double JvDotJv = FVector::DotProduct(Jv, Jv);

			if (JvDotJv < KINDA_SMALL_NUMBER)
				continue; // twist joint — can't affect EE position

			// Scalar DLS: dq = (Jv . dp) / (Jv . Jv + lambda^2)
			double dqRad = JvDotDp / (JvDotJv + Lambda2);

			// Step clamp (in radians, like DLS)
			if (StepClipRad > 0.0)
				dqRad = FMath::Clamp(dqRad, -StepClipRad, StepClipRad);

			// Apply gain
			dqRad *= (double)AngleGain;

			// Convert to degrees, apply joint limits
			float NewAngle = Angles[i] + FMath::RadiansToDegrees((float)dqRad);
			NewAngle = ApplyAngleLimits(NewAngle, JointLimitsDeg[i], LimitEscapeDeg);

			// Actual applied delta (may differ due to limits)
			const float ActualDeltaDeg = NewAngle - Angles[i];
			const double ActualDqRad = FMath::DegreesToRadians(ActualDeltaDeg);
			Angles[i] = NewAngle;

			// Subtract this joint's predicted EE movement from residual
			// so the next joint sees the corrected error
			dpResidual -= Jv * (float)ActualDqRad;
		}
	}

	// ------------------------------------------------------------------
	// Optional: CCD-style orientation refinement using the last few joints
	// ------------------------------------------------------------------
	if (bSolveOrientation && OrientationIterations > 0 && OrientationGain > 0.0f)
	{
		const int32 StartIdx = FMath::Max(0, N - 3);
		const double OriLambda2 = 0.01; // small damping for orientation

		for (int32 OIter = 0; OIter < OrientationIterations; OIter++)
		{
			ComputeChainKinematics(BaseTransform, Angles, JointLocalTransforms,
				JointAxesLocal, EndEffectorOffset, JointPosWorld, JointAxisWorld, EEWorld);

			// Rotation error as axis-angle vector in world space (same as DLS)
			const Eigen::Vector3d drot = RotationErrorAxisAngleRad(
				EEWorld.GetRotation().GetNormalized(),
				TargetEndEffectorWorld.GetRotation().GetNormalized());

			if (drot.norm() < FMath::DegreesToRadians(0.5f))
				break;

			const FVector dr((float)drot.x(), (float)drot.y(), (float)drot.z()); // rad

			for (int32 i = N - 1; i >= StartIdx; i--)
			{
				const FVector AxisW = JointAxisWorld[i].GetSafeNormal();
				if (AxisW.IsNearlyZero())
					continue;

				// Jw = axis (unit), so Jw.Jw = 1
				// dq = (Jw . dr) / (1 + lambda^2)
				const double JwDotDr = FVector::DotProduct(AxisW, dr); // rad
				const double dqRad = JwDotDr / (1.0 + OriLambda2);

				float DeltaDeg = FMath::RadiansToDegrees((float)dqRad) * OrientationGain;
				DeltaDeg = FMath::Clamp(DeltaDeg, -MaxAngleStepDeg, MaxAngleStepDeg);

				float NewAngle = Angles[i] + DeltaDeg;
				NewAngle = ApplyAngleLimits(NewAngle, JointLimitsDeg[i], LimitEscapeDeg);
				Angles[i] = NewAngle;
			}
		}
	}

	// Final FK for error metrics
	ComputeChainKinematics(BaseTransform, Angles, JointLocalTransforms,
		JointAxesLocal, EndEffectorOffset, JointPosWorld, JointAxisWorld, EEWorld);

	Result.JointAngles = Angles;
	Result.PositionError = FVector::Dist(EEWorld.GetLocation(), TargetPos);
	Result.RotationError = RotationErrorDeg(EEWorld.GetRotation(), TargetEndEffectorWorld.GetRotation());

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
