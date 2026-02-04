// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsIKLibrary.h"
#include "DrawDebugHelpers.h"

// Forward Kinematics: Rotate then Translate
FTransform URammsIKLibrary::ComputeForwardKinematics(
	const FTransform& BaseTransform,
	const TArray<float>& JointAngles,
	const TArray<FTransform>& JointLocalTransforms,
	const TArray<FVector>& JointAxes,
	const FTransform& EndEffectorOffset,
	bool bDebugLog)
{
	const int32 NumJoints = JointAngles.Num();
	
	// Start at base
	FTransform CurrentTransform = BaseTransform;
	
	if (bDebugLog)
	{
		UE_LOG(LogTemp, Log, TEXT("[FK] Base: Loc=(%.1f, %.1f, %.1f) Rot(P:%.1f, Y:%.1f, R:%.1f)"),
			CurrentTransform.GetLocation().X, CurrentTransform.GetLocation().Y, CurrentTransform.GetLocation().Z,
			CurrentTransform.Rotator().Pitch, CurrentTransform.Rotator().Yaw, CurrentTransform.Rotator().Roll);
	}
	
	// Apply each joint transformation
	for (int32 i = 0; i < NumJoints; i++)
	{
		// 1. ROTATE at current pivot
		FVector WorldAxis = CurrentTransform.TransformVectorNoScale(JointAxes[i]).GetSafeNormal();
		float AngleRad = FMath::DegreesToRadians(JointAngles[i]);
		FQuat JointRotation(WorldAxis, AngleRad);
		CurrentTransform.SetRotation(JointRotation * CurrentTransform.GetRotation());
		
		// 2. TRANSLATE to next pivot
		FVector LocalOffset = JointLocalTransforms[i].GetTranslation();
		FVector WorldOffset = CurrentTransform.TransformVectorNoScale(LocalOffset);
		CurrentTransform.AddToTranslation(WorldOffset);
		
		if (bDebugLog)
		{
			UE_LOG(LogTemp, Log, TEXT("[FK] Joint[%d]: Angle=%.1f deg, Offset=(%.1f,%.1f,%.1f) -> Pos=(%.1f,%.1f,%.1f)"),
				i, JointAngles[i], LocalOffset.X, LocalOffset.Y, LocalOffset.Z,
				CurrentTransform.GetLocation().X, CurrentTransform.GetLocation().Y, CurrentTransform.GetLocation().Z);
		}
	}
	
	// Apply end effector offset
	FVector EELocalOffset = EndEffectorOffset.GetTranslation();
	FVector EEWorldOffset = CurrentTransform.TransformVectorNoScale(EELocalOffset);
	CurrentTransform.AddToTranslation(EEWorldOffset);
	CurrentTransform.SetRotation(EndEffectorOffset.GetRotation() * CurrentTransform.GetRotation());
	
	if (bDebugLog)
	{
		UE_LOG(LogTemp, Log, TEXT("[FK] EE: Loc=(%.1f, %.1f, %.1f)"),
			CurrentTransform.GetLocation().X, CurrentTransform.GetLocation().Y, CurrentTransform.GetLocation().Z);
	}
	
	return CurrentTransform;
}

// Damped Least Squares IK Solver
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
	Result.JointAngles = JointAngles;
	
	const int32 N = JointAngles.Num();
	if (N == 0 || N != JointLocalTransforms.Num() || N != JointAxes.Num() || N != JointLimits.Num())
	{
		UE_LOG(LogTemp, Error, TEXT("[IK] Invalid input sizes"));
		return Result;
	}
	
	// Build task space selection matrix W (TaskDim x 6)
	int32 TaskDim = 0;
	TArray<int32> ActiveDOFs;
	for (int32 i = 0; i < 6 && i < TaskSpaceMask.Num(); i++)
	{
		if (TaskSpaceMask[i])
		{
			ActiveDOFs.Add(i);
			TaskDim++;
		}
	}
	if (TaskDim == 0) TaskDim = 6; // Default to full 6D
	
	Eigen::MatrixXd W = Eigen::MatrixXd::Zero(TaskDim, 6);
	for (int32 i = 0; i < TaskDim; i++)
	{
		int32 dof = (ActiveDOFs.Num() > 0) ? ActiveDOFs[i] : i;
		W(i, dof) = 1.0;
	}
	
	// Joint weights (diagonal matrix Wq)
	Eigen::MatrixXd Wq = Eigen::MatrixXd::Identity(N, N);
	if (JointWeights.Num() == N)
	{
		for (int32 i = 0; i < N; i++)
		{
			Wq(i, i) = 1.0 / FMath::Max(JointWeights[i], 0.01f);
		}
	}
	
	// Task scaling matrix S (6x6) - scales cm to meters and keeps rotation in radians
	Eigen::MatrixXd S = Eigen::MatrixXd::Identity(6, 6);
	S(0, 0) = S(1, 1) = S(2, 2) = 0.01; // Position: cm -> m
	// Rotation already in radians, no scaling needed
	
	// Damping matrix
	const double lambda = FMath::Max(0.001, (double)DampingFactor);
	Eigen::MatrixXd Lambda = Eigen::MatrixXd::Identity(TaskDim, TaskDim) * (lambda * lambda);
	
	// Null-space bias (if enabled)
	bool bUseNullSpace = (NullSpaceGain > 1e-6f) && (NullSpaceBias.Num() == N) && (N > TaskDim);
	Eigen::VectorXd q0_rad(N);
	if (bUseNullSpace)
	{
		for (int32 i = 0; i < N; i++)
		{
			q0_rad(i) = FMath::DegreesToRadians(NullSpaceBias[i]);
		}
	}
	
	// IK iteration loop
	TArray<float> CurrentAngles = JointAngles;
	for (int32 Iter = 0; Iter < MaxIterations; Iter++)
	{
		// Compute current end effector pose
		FTransform CurrentEE = ComputeForwardKinematics(
			BaseTransform, CurrentAngles, JointLocalTransforms, JointAxes, EndEffectorOffset, false);
		
		// Compute task space error
		FVector PositionError = TargetTransform.GetLocation() - CurrentEE.GetLocation();
		FQuat RotationErrorQuat = TargetTransform.GetRotation() * CurrentEE.GetRotation().Inverse();
		
		// Enforce shortest arc (flip quaternion if needed)
		if ((TargetTransform.GetRotation() | CurrentEE.GetRotation()) < 0.0f)
		{
			RotationErrorQuat = FQuat(-RotationErrorQuat.X, -RotationErrorQuat.Y, -RotationErrorQuat.Z, -RotationErrorQuat.W);
		}
		
		// Convert rotation error to axis-angle (approximate)
		FVector RotationErrorVec = FVector::ZeroVector;
		{
			FQuat q = RotationErrorQuat;
			q.Normalize();
			float w = FMath::Clamp(q.W, -1.0f, 1.0f);
			FVector v(q.X, q.Y, q.Z);
			float n = v.Size();
			if (n > 1e-8f)
			{
				float angle = 2.0f * FMath::Atan2(n, w);
				RotationErrorVec = (angle / n) * v;
			}
			else
			{
				RotationErrorVec = 2.0f * v; // Small angle approx
			}
		}
		
		// Check convergence
		float PosErr = PositionError.Size();
		float RotErr = FMath::RadiansToDegrees(RotationErrorVec.Size());
		Result.PositionError = PosErr;
		Result.RotationError = RotErr;
		Result.IterationsUsed = Iter + 1;
		
		if (PosErr < PositionTolerance && RotErr < RotationTolerance)
		{
			Result.bSuccess = true;
			break;
		}
		
		// Build 6D error vector
		Eigen::VectorXd e_full(6);
		e_full << PositionError.X, PositionError.Y, PositionError.Z,
		          RotationErrorVec.X, RotationErrorVec.Y, RotationErrorVec.Z;
		
		// Apply task scaling and selection
		Eigen::VectorXd e = W * (S * e_full);
		
		// Compute numerical Jacobian (6 x N)
		Eigen::MatrixXd J_full(6, N);
		J_full.setZero();
		
		const double h = 1e-4; // Perturbation in radians
		for (int32 j = 0; j < N; j++)
		{
			TArray<float> AnglesPerturbed = CurrentAngles;
			AnglesPerturbed[j] += FMath::RadiansToDegrees(h);
			
			FTransform EE_perturbed = ComputeForwardKinematics(
				BaseTransform, AnglesPerturbed, JointLocalTransforms, JointAxes, EndEffectorOffset, false);
			
			// Position derivative
			FVector dPos = (EE_perturbed.GetLocation() - CurrentEE.GetLocation()) / h;
			J_full(0, j) = dPos.X;
			J_full(1, j) = dPos.Y;
			J_full(2, j) = dPos.Z;
			
			// Rotation derivative (quaternion difference)
			FQuat dQuat = EE_perturbed.GetRotation() * CurrentEE.GetRotation().Inverse();
			FQuat dQuatNorm = dQuat;
			dQuatNorm.Normalize();
			float w = FMath::Clamp(dQuatNorm.W, -1.0f, 1.0f);
			FVector v(dQuatNorm.X, dQuatNorm.Y, dQuatNorm.Z);
			float n = v.Size();
			FVector omega = FVector::ZeroVector;
			if (n > 1e-8f)
			{
				float angle = 2.0f * FMath::Atan2(n, w);
				omega = (angle / n) * v / h;
			}
			else
			{
				omega = 2.0f * v / h;
			}
			
			J_full(3, j) = omega.X;
			J_full(4, j) = omega.Y;
			J_full(5, j) = omega.Z;
		}
		
		// Apply task scaling and selection to Jacobian
		Eigen::MatrixXd J = W * (S * J_full);
		
		// Solve for joint velocities using DLS with joint weights
		// dq = Wq * J^T * (J * Wq * J^T + λ²I)^-1 * e
		Eigen::MatrixXd JWq = J * Wq;
		Eigen::MatrixXd A = JWq * J.transpose() + Lambda; // (TaskDim x TaskDim)
		Eigen::MatrixXd JTWinv = Wq * J.transpose(); // (N x TaskDim)
		
		// Solve using LDLT (numerically stable)
		Eigen::LDLT<Eigen::MatrixXd> ldlt(A);
		Eigen::MatrixXd A_inv = ldlt.solve(Eigen::MatrixXd::Identity(TaskDim, TaskDim));
		Eigen::VectorXd dq_rad = JTWinv * (A_inv * e);
		
		// Null-space projection (only if redundant)
		if (bUseNullSpace)
		{
			Eigen::MatrixXd J_pinv = JTWinv * A_inv; // (N x TaskDim)
			Eigen::MatrixXd Nullspace = Eigen::MatrixXd::Identity(N, N) - (J_pinv * J);
			
			Eigen::VectorXd q_rad(N);
			for (int32 i = 0; i < N; i++)
			{
				q_rad(i) = FMath::DegreesToRadians(CurrentAngles[i]);
			}
			
			Eigen::VectorXd dq_null = NullSpaceGain * (q0_rad - q_rad);
			dq_rad += Nullspace * dq_null;
		}
		
		// Clip step size
		for (int32 i = 0; i < N; i++)
		{
			dq_rad(i) = FMath::Clamp((float)dq_rad(i), -StepClip, StepClip);
		}
		
		// Update joint angles and apply limits
		for (int32 i = 0; i < N; i++)
		{
			float DeltaDeg = FMath::RadiansToDegrees((float)dq_rad(i));
			CurrentAngles[i] = FMath::Clamp(
				CurrentAngles[i] + DeltaDeg,
				JointLimits[i].X,
				JointLimits[i].Y
			);
		}
	}
	
	Result.JointAngles = CurrentAngles;
	return Result;
}

// Utility functions
float URammsIKLibrary::ClampJointAngle(float Angle, float Min, float Max)
{
	return FMath::Clamp(Angle, Min, Max);
}

void URammsIKLibrary::CalculateTransformError(
	const FTransform& Transform1,
	const FTransform& Transform2,
	float& OutPositionError,
	float& OutRotationError)
{
	OutPositionError = (Transform2.GetLocation() - Transform1.GetLocation()).Size();
	
	FQuat Q1 = Transform1.GetRotation();
	FQuat Q2 = Transform2.GetRotation();
	FQuat DeltaQ = Q2 * Q1.Inverse();
	OutRotationError = FMath::RadiansToDegrees(DeltaQ.GetAngle());
}

float URammsIKLibrary::NormalizeAngle(float Angle)
{
	while (Angle > 180.0f) Angle -= 360.0f;
	while (Angle < -180.0f) Angle += 360.0f;
	return Angle;
}

// Legacy FABRIK and Jacobian Transpose (stub implementations)
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
	// Legacy - not implemented
	FIKSolveResult Result;
	Result.bSuccess = false;
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
	// Legacy - not implemented
	FIKSolveResult Result;
	Result.bSuccess = false;
	return Result;
}
