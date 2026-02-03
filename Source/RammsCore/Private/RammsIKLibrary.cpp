// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsIKLibrary.h"

// Forward declaration of internal helper
static FTransform ComputeForwardKinematicsInternal(
	const FTransform& BaseTransform,
	const TArray<float>& JointAnglesRad,
	const TArray<FTransform>& JointLocalTransforms,
	const TArray<FVector>& JointAxes,
	const FTransform& EndEffectorOffset);

FIKSolveResult URammsIKLibrary::SolveIK_FABRIK(
	const TArray<FVector>& JointPositions,
	TArray<float>& JointAngles,
	const FTransform& TargetTransform,
	const TArray<FVector2D>& JointLimits,
	int32 MaxIterations,
	float PositionTolerance,
	float RotationTolerance,
	float StepSize)
{
	FIKSolveResult Result;
	Result.bSuccess = false;

	if (JointPositions.Num() < 2 || JointPositions.Num() != JointLimits.Num())
	{
		return Result;
	}

	// FABRIK algorithm - simplified for position only
	TArray<FVector> Positions = JointPositions;
	FVector TargetPosition = TargetTransform.GetLocation();
	
	// Calculate segment lengths
	TArray<float> SegmentLengths;
	for (int32 i = 0; i < Positions.Num() - 1; i++)
	{
		SegmentLengths.Add(FVector::Dist(Positions[i], Positions[i + 1]));
	}

	FVector RootPosition = Positions[0];

	for (int32 Iter = 0; Iter < MaxIterations; Iter++)
	{
		// Check convergence
		float DistanceToTarget = FVector::Dist(Positions.Last(), TargetPosition);
		if (DistanceToTarget < PositionTolerance)
		{
			Result.bSuccess = true;
			Result.IterationsUsed = Iter + 1;
			break;
		}

		// Forward reaching - start from end effector
		Positions.Last() = FMath::Lerp(Positions.Last(), TargetPosition, StepSize);
		
		for (int32 i = Positions.Num() - 2; i >= 0; i--)
		{
			FVector Direction = (Positions[i] - Positions[i + 1]).GetSafeNormal();
			Positions[i] = Positions[i + 1] + Direction * SegmentLengths[i];
		}

		// Backward reaching - start from root
		Positions[0] = RootPosition;
		
		for (int32 i = 0; i < Positions.Num() - 1; i++)
		{
			FVector Direction = (Positions[i + 1] - Positions[i]).GetSafeNormal();
			Positions[i + 1] = Positions[i] + Direction * SegmentLengths[i];
		}
	}

	// Convert positions to joint angles (simplified - needs proper FK/IK chain)
	// This is a placeholder - real implementation needs joint axis information
	Result.JointAngles = JointAngles; // Keep original angles for now
	
	// Calculate final errors
	CalculateTransformError(
		FTransform(Positions.Last()),
		TargetTransform,
		Result.PositionError,
		Result.RotationError);

	return Result;
}

FIKSolveResult URammsIKLibrary::SolveIK_JacobianTranspose(
	TArray<float>& JointAngles,
	const TArray<FTransform>& JointTransforms,
	const TArray<FVector>& JointAxes,
	const FTransform& EndEffectorTransform,
	const FTransform& TargetTransform,
	const TArray<FVector2D>& JointLimits,
	int32 MaxIterations,
	float PositionTolerance,
	float RotationTolerance,
	float StepSize)
{
	FIKSolveResult Result;
	Result.bSuccess = false;

	if (JointAngles.Num() != JointTransforms.Num() || 
		JointAngles.Num() != JointAxes.Num() ||
		JointAngles.Num() != JointLimits.Num())
	{
		return Result;
	}

	int32 NumJoints = JointAngles.Num();
	TArray<float> CurrentAngles = JointAngles;

	// Calculate position and orientation error (use actual end effector state from skeletal mesh)
	FVector PositionError = TargetTransform.GetLocation() - EndEffectorTransform.GetLocation();
	FQuat RotationError = TargetTransform.GetRotation() * EndEffectorTransform.GetRotation().Inverse();
	FVector RotationAxis;
	float RotationAngle;
	RotationError.ToAxisAndAngle(RotationAxis, RotationAngle);
	FVector OrientationError = RotationAxis * RotationAngle;

	// Check if already at target
	float PosErrorMag = PositionError.Size();
	float RotErrorMag = FMath::RadiansToDegrees(FMath::Abs(RotationAngle));
	
	if (PosErrorMag < PositionTolerance && RotErrorMag < RotationTolerance)
	{
		Result.bSuccess = true;
		Result.IterationsUsed = 0;
		Result.PositionError = PosErrorMag;
		Result.RotationError = RotErrorMag;
		Result.JointAngles = CurrentAngles;
		return Result;
	}

	// Build Jacobian using ACTUAL skeletal mesh joint transforms and axes
	// For each joint, calculate how end effector velocity relates to joint velocity
	TArray<FVector> PositionJacobian; // Linear velocity columns
	TArray<FVector> OrientationJacobian; // Angular velocity columns

	for (int32 i = 0; i < NumJoints; i++)
	{
		// Use the provided joint axis (already in world space, accounts for Twist/Swing1/Swing2)
		FVector JointAxis = JointAxes[i].GetSafeNormal();
		
		// Vector from joint to end effector
		FVector JointToEE = EndEffectorTransform.GetLocation() - JointTransforms[i].GetLocation();
		
		// Linear Jacobian column: ω × r = axis × (endeffector - joint)
		// This gives the linear velocity of end effector when joint rotates
		FVector LinearColumn = FVector::CrossProduct(JointAxis, JointToEE);
		PositionJacobian.Add(LinearColumn);
		
		// Angular Jacobian column: just the rotation axis
		// This gives the angular velocity of end effector when joint rotates
		OrientationJacobian.Add(JointAxis);
	}

	// Compute joint angle changes using damped Jacobian transpose method
	// Δθ = α * J^T * e where α is step size, J^T is Jacobian transpose, e is error
	// This is ONE step toward the target - physics will handle convergence over multiple frames
	
	// Normalize Jacobian columns for more stable behavior
	float MaxJacobianMag = 0.0f;
	for (int32 i = 0; i < NumJoints; i++)
	{
		float Mag = PositionJacobian[i].Size();
		if (Mag > MaxJacobianMag)
			MaxJacobianMag = Mag;
	}
	
	// Avoid division by zero
	if (MaxJacobianMag < 0.01f)
		MaxJacobianMag = 1.0f;
	
	TArray<float> AngleDelta;
	AngleDelta.SetNumZeroed(NumJoints);
	
	for (int32 i = 0; i < NumJoints; i++)
	{
		// Position contribution: how much this joint affects position error
		// J^T[i] · e_position
		float PositionContribution = FVector::DotProduct(PositionJacobian[i], PositionError);
		
		// Orientation contribution: how much this joint affects orientation error
		// J^T[i] · e_orientation
		float OrientationContribution = FVector::DotProduct(OrientationJacobian[i], OrientationError);
		
		// Normalize position contribution by arm reach to get consistent scaling
		PositionContribution /= MaxJacobianMag;
		
		// Combine contributions
		// Position is in cm, convert to pseudo-radians (1cm error ~= 0.01 radians needed)
		// Orientation is already in radians
		float CombinedError = (PositionContribution * 0.01f) + (OrientationContribution * 0.2f);
		
		// Apply step size and convert to degrees
		AngleDelta[i] = StepSize * FMath::RadiansToDegrees(CombinedError);
		
		// Clamp individual joint deltas to prevent huge steps
		AngleDelta[i] = FMath::Clamp(AngleDelta[i], -10.0f * StepSize, 10.0f * StepSize);
	}

	// Apply angle deltas with limits
	for (int32 i = 0; i < NumJoints; i++)
	{
		CurrentAngles[i] += AngleDelta[i];
		CurrentAngles[i] = ClampJointAngle(CurrentAngles[i], JointLimits[i].X, JointLimits[i].Y);
	}

	Result.JointAngles = CurrentAngles;
	Result.IterationsUsed = 1; // Single iteration per frame - physics handles convergence
	Result.PositionError = PosErrorMag;
	Result.RotationError = RotErrorMag;
	Result.bSuccess = (PosErrorMag < PositionTolerance * 5.0f); // Consider "success" if reasonably close

	return Result;
}

float URammsIKLibrary::ClampJointAngle(float Angle, float Min, float Max)
{
	// Normalize first
	float Normalized = NormalizeAngle(Angle);
	return FMath::Clamp(Normalized, Min, Max);
}

void URammsIKLibrary::CalculateTransformError(
	const FTransform& Transform1,
	const FTransform& Transform2,
	float& OutPositionError,
	float& OutRotationError)
{
	// Position error in cm
	OutPositionError = FVector::Dist(Transform1.GetLocation(), Transform2.GetLocation());

	// Rotation error in degrees
	FQuat Q1 = Transform1.GetRotation();
	FQuat Q2 = Transform2.GetRotation();
	FQuat DeltaQ = Q2 * Q1.Inverse();
	
	float Angle;
	FVector Axis;
	DeltaQ.ToAxisAndAngle(Axis, Angle);
	
	OutRotationError = FMath::RadiansToDegrees(FMath::Abs(Angle));
}

float URammsIKLibrary::NormalizeAngle(float Angle)
{
	float Normalized = FMath::Fmod(Angle, 360.0f);
	if (Normalized > 180.0f)
		Normalized -= 360.0f;
	else if (Normalized < -180.0f)
		Normalized += 360.0f;
	return Normalized;
}
// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsIKLibrary.h"

THIRD_PARTY_INCLUDES_START
#include <Eigen/Dense>
THIRD_PARTY_INCLUDES_END

FTransform URammsIKLibrary::ComputeForwardKinematics(
	const FTransform& BaseTransform,
	const TArray<float>& JointAngles,
	const TArray<FTransform>& JointLocalTransforms,
	const TArray<FVector>& JointAxes,
	const FTransform& EndEffectorOffset,
	bool bDebugLog)
{
	const int32 NumJoints = JointAngles.Num();
	if (NumJoints != JointLocalTransforms.Num() || NumJoints != JointAxes.Num())
	{
		UE_LOG(LogTemp, Error, TEXT("FK: Mismatched array sizes"));
		return BaseTransform;
	}

	// Convert angles from degrees to radians
	TArray<float> JointAnglesRad;
	JointAnglesRad.SetNum(NumJoints);
	for (int32 i = 0; i < NumJoints; i++)
	{
		JointAnglesRad[i] = FMath::DegreesToRadians(JointAngles[i]);
	}

	return ComputeForwardKinematicsInternal(BaseTransform, JointAnglesRad, JointLocalTransforms, JointAxes, EndEffectorOffset, bDebugLog);
}

// Internal helper that takes radians (static free function)
static FTransform ComputeForwardKinematicsInternal(
	const FTransform& BaseTransform,
	const TArray<float>& JointAnglesRad,
	const TArray<FTransform>& JointLocalTransforms,
	const TArray<FVector>& JointAxes,
	const FTransform& EndEffectorOffset,
	bool bDebugLog)
{
	// Start from base
	FTransform CurrentTransform = BaseTransform;
	
	if (bDebugLog)
	{
		UE_LOG(LogTemp, Log, TEXT("[FK] Starting from base: (%.1f, %.1f, %.1f), Rot(P:%.1f, Y:%.1f, R:%.1f)"),
			CurrentTransform.GetLocation().X, CurrentTransform.GetLocation().Y, CurrentTransform.GetLocation().Z,
			CurrentTransform.Rotator().Pitch, CurrentTransform.Rotator().Yaw, CurrentTransform.Rotator().Roll);
	}

	// Chain through each joint
	for (int32 i = 0; i < JointAnglesRad.Num(); i++)
	{
		// First apply the local joint transform (move to the joint's location)
		CurrentTransform = CurrentTransform * JointLocalTransforms[i];
		
		if (bDebugLog)
		{
			FVector AfterMoveLoc = CurrentTransform.GetLocation();
			UE_LOG(LogTemp, Log, TEXT("[FK]   Joint[%d] after offset: (%.1f, %.1f, %.1f)"), 
				i, AfterMoveLoc.X, AfterMoveLoc.Y, AfterMoveLoc.Z);
		}
		
		// Now we're at the joint location - apply rotation here
		// JointAxes[i] is in the coordinate frame of this joint (after the offset)
		// Transform the axis to world space
		FVector WorldAxis = CurrentTransform.TransformVectorNoScale(JointAxes[i]);
		
		// Apply joint rotation around world-space axis
		FQuat JointRotation(WorldAxis, JointAnglesRad[i]);
		CurrentTransform.SetRotation(JointRotation * CurrentTransform.GetRotation());
	}

	// Apply end effector offset
	FTransform EndEffectorTransform = EndEffectorOffset * CurrentTransform;

	return EndEffectorTransform;
}

FIKSolveResult URammsIKLibrary::SolveIK_DLS(
	TArray<float>& JointAngles,
	const FTransform& BaseTransform,
	const TArray<FTransform>& JointLocalTransforms,
	const TArray<FVector>& JointAxes,
	const FTransform& EndEffectorOffset,
	const FTransform& TargetTransform,
	const TArray<FVector2D>& JointLimits,
	const TArray<bool>& TaskSpaceMask,
	const TArray<float>& JointWeights,
	float DampingFactor,
	float StepClip,
	int32 MaxIterations,
	float PositionTolerance,
	float RotationTolerance,
	const TArray<float>& NullSpaceBias,
	float NullSpaceGain)
{
	FIKSolveResult Result;
	Result.bSuccess = false;
	Result.IterationsUsed = 0;
	Result.PositionError = 0.0f;
	Result.RotationError = 0.0f;

	const int32 NumJoints = JointAngles.Num();
	if (NumJoints == 0 || NumJoints != JointLocalTransforms.Num() || 
		NumJoints != JointAxes.Num() || NumJoints != JointLimits.Num())
	{
		UE_LOG(LogTemp, Error, TEXT("IK_DLS: Mismatched input array sizes"));
		return Result;
	}

	// Convert joint angles from degrees to radians
	Eigen::VectorXd q(NumJoints);
	TArray<float> JointAnglesRad;
	JointAnglesRad.SetNum(NumJoints);
	for (int32 i = 0; i < NumJoints; i++)
	{
		JointAnglesRad[i] = FMath::DegreesToRadians(JointAngles[i]);
		q(i) = JointAnglesRad[i];
	}

	// Convert joint limits to radians
	Eigen::VectorXd q_min(NumJoints);
	Eigen::VectorXd q_max(NumJoints);
	for (int32 i = 0; i < NumJoints; i++)
	{
		q_min(i) = FMath::DegreesToRadians(JointLimits[i].X);
		q_max(i) = FMath::DegreesToRadians(JointLimits[i].Y);
	}

	// Set up task-space dimensions
	int32 TaskDim = 6;
	if (TaskSpaceMask.Num() == 6)
	{
		TaskDim = 0;
		for (bool bMask : TaskSpaceMask)
		{
			if (bMask) TaskDim++;
		}
	}

	// Task-space mask matrix (TaskDim x 6)
	Eigen::MatrixXd W = Eigen::MatrixXd::Zero(TaskDim, 6);
	int32 MaskRow = 0;
	for (int32 i = 0; i < 6 && i < TaskSpaceMask.Num(); i++)
	{
		if (TaskSpaceMask[i])
		{
			W(MaskRow, i) = 1.0;
			MaskRow++;
		}
	}

	// Prepare null-space bias if provided
	Eigen::VectorXd q0(NumJoints);
	bool bUseNullSpace = (NullSpaceBias.Num() == NumJoints) && (NullSpaceGain > 1e-6);
	if (bUseNullSpace)
	{
		for (int32 i = 0; i < NumJoints; i++)
		{
			q0(i) = FMath::DegreesToRadians(NullSpaceBias[i]);
		}
	}

	// Joint weight matrix Wq (diagonal)
	Eigen::MatrixXd Wq = Eigen::MatrixXd::Identity(NumJoints, NumJoints);
	if (JointWeights.Num() == NumJoints)
	{
		for (int32 i = 0; i < NumJoints; i++)
		{
			// Higher weight = prefer not to move this joint
			// Invert for DLS formulation
			Wq(i, i) = 1.0 / FMath::Max(JointWeights[i], 0.01);
		}
	}

	// DLS damping matrix
	Eigen::MatrixXd Lambda = Eigen::MatrixXd::Identity(TaskDim, TaskDim) * (DampingFactor * DampingFactor);

	// Numerical Jacobian perturbation size (radians)
	const double h = 1e-4;

	// ===== MAIN IK LOOP =====
	for (int32 Iter = 0; Iter < MaxIterations; Iter++)
	{
		// ===== 1. Compute forward kinematics for current configuration =====
		FTransform CurrentEE = ComputeForwardKinematicsInternal(
			BaseTransform, JointAnglesRad, JointLocalTransforms, JointAxes, EndEffectorOffset, false);

		// ===== 2. Compute task-space error =====
		FVector PositionError = TargetTransform.GetLocation() - CurrentEE.GetLocation();
		
		// Orientation error using quaternion log map
		FQuat CurrentQuat = CurrentEE.GetRotation();
		FQuat TargetQuat = TargetTransform.GetRotation();
		FQuat QuatError = TargetQuat * CurrentQuat.Inverse();
		QuatError.Normalize();
		
		// Convert to axis-angle (approximation of quaternion log)
		FVector OrientationError = FVector::ZeroVector;
		if (FMath::Abs(QuatError.W) < 0.9999)
		{
			float Angle = 2.0f * FMath::Acos(FMath::Clamp(QuatError.W, -1.0f, 1.0f));
			float SinHalfAngle = FMath::Sqrt(1.0f - QuatError.W * QuatError.W);
			if (SinHalfAngle > 1e-6f)
			{
				OrientationError = FVector(QuatError.X, QuatError.Y, QuatError.Z) * (Angle / SinHalfAngle);
			}
		}

		// Build 6D error vector
		Eigen::VectorXd e_full(6);
		e_full(0) = PositionError.X;
		e_full(1) = PositionError.Y;
		e_full(2) = PositionError.Z;
		e_full(3) = OrientationError.X;
		e_full(4) = OrientationError.Y;
		e_full(5) = OrientationError.Z;

		// Apply task-space masking
		Eigen::VectorXd e = W * e_full;

		// ===== 3. Check convergence =====
		float PosErrorMag = PositionError.Size();
		float RotErrorMag = FMath::RadiansToDegrees(OrientationError.Size());
		
		Result.PositionError = PosErrorMag;
		Result.RotationError = RotErrorMag;
		Result.IterationsUsed = Iter + 1;

		if (PosErrorMag < PositionTolerance && RotErrorMag < RotationTolerance)
		{
			Result.bSuccess = true;
			break;
		}

		// ===== 4. Compute numerical Jacobian =====
		Eigen::MatrixXd J_full(6, NumJoints);
		
		for (int32 j = 0; j < NumJoints; j++)
		{
			// Perturb joint j by +h
			TArray<float> q_perturbed = JointAnglesRad;
			q_perturbed[j] += h;

			// Compute FK at perturbed configuration
			FTransform EE_perturbed = ComputeForwardKinematicsInternal(
				BaseTransform, q_perturbed, JointLocalTransforms, JointAxes, EndEffectorOffset, false);

			// Position derivative
			FVector dPos = (EE_perturbed.GetLocation() - CurrentEE.GetLocation()) / h;
			J_full(0, j) = dPos.X;
			J_full(1, j) = dPos.Y;
			J_full(2, j) = dPos.Z;

			// Orientation derivative (quaternion difference)
			FQuat CurrentQuatJ = CurrentEE.GetRotation();
			FQuat PerturbedQuatJ = EE_perturbed.GetRotation();
			FQuat dQuat = PerturbedQuatJ * CurrentQuatJ.Inverse();
			dQuat.Normalize();

			// Convert to axis-angle
			FVector dRot = FVector::ZeroVector;
			if (FMath::Abs(dQuat.W) < 0.9999)
			{
				float dAngle = 2.0f * FMath::Acos(FMath::Clamp(dQuat.W, -1.0f, 1.0f));
				float dSinHalfAngle = FMath::Sqrt(1.0f - dQuat.W * dQuat.W);
				if (dSinHalfAngle > 1e-6f)
				{
					dRot = FVector(dQuat.X, dQuat.Y, dQuat.Z) * (dAngle / dSinHalfAngle);
				}
			}
			dRot /= h;

			J_full(3, j) = dRot.X;
			J_full(4, j) = dRot.Y;
			J_full(5, j) = dRot.Z;
		}

		// Apply task-space selection to Jacobian
		Eigen::MatrixXd J = W * J_full;

		// ===== 5. Compute DLS pseudo-inverse with joint weights =====
		// Formula: Δq = Wq * J^T * (J * Wq * J^T + λ²I)^-1 * e
		Eigen::MatrixXd JWq = J * Wq;
		Eigen::MatrixXd JWqJT = JWq * J.transpose() + Lambda;
		Eigen::MatrixXd J_pinv = Wq * J.transpose() * JWqJT.inverse();

		// ===== 6. Compute joint velocity =====
		Eigen::VectorXd dq = J_pinv * e;

		// ===== 7. Null-space projection (for redundant manipulators) =====
		if (bUseNullSpace)
		{
			// Null-space projector: N = I - J_pinv * J
			Eigen::MatrixXd N = Eigen::MatrixXd::Identity(NumJoints, NumJoints) - J_pinv * J;
			
			// Secondary objective: minimize (q - q0)
			Eigen::VectorXd dq_null = NullSpaceGain * (q0 - q);
			
			// Add null-space component
			dq += N * dq_null;
		}

		// ===== 8. Clip step size (prevents large jumps) =====
		for (int32 i = 0; i < NumJoints; i++)
		{
			dq(i) = FMath::Clamp(dq(i), -StepClip, StepClip);
		}

		// ===== 9. Apply joint limits with velocity limiting =====
		for (int32 i = 0; i < NumJoints; i++)
		{
			// Compute new angle
			double q_new = q(i) + dq(i);

			// Hard clamp to limits
			q_new = FMath::Clamp(q_new, q_min(i), q_max(i));

			// Apply soft limiting near boundaries (slow down approach)
			const double LimitMargin = 0.1; // radians (~5.7 degrees)
			double DistToMin = q_new - q_min(i);
			double DistToMax = q_max(i) - q_new;

			if (DistToMin < LimitMargin && dq(i) < 0)
			{
				// Approaching min limit
				double Scale = DistToMin / LimitMargin;
				dq(i) *= Scale;
				q_new = q(i) + dq(i);
			}
			else if (DistToMax < LimitMargin && dq(i) > 0)
			{
				// Approaching max limit
				double Scale = DistToMax / LimitMargin;
				dq(i) *= Scale;
				q_new = q(i) + dq(i);
			}

			q(i) = q_new;
			JointAnglesRad[i] = q_new;
		}
	}

	// Convert result back to degrees
	for (int32 i = 0; i < NumJoints; i++)
	{
		JointAngles[i] = FMath::RadiansToDegrees(JointAnglesRad[i]);
	}

	Result.JointAngles = JointAngles;
	return Result;
}

// Legacy Jacobian Transpose method (deprecated - use DLS instead)
