// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsSkeletalPoseComponent.h"
#include "Components/PoseableMeshComponent.h"
#include "Engine/SkeletalMesh.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "PhysicsEngine/PhysicsConstraintTemplate.h"
#include "PhysicsEngine/ConstraintInstance.h"
#include "RammsJointPoseAsset.h"

namespace
{
	/**
	 * Find which cardinal axis (X, Y, Z) most closely aligns with a direction vector.
	 * Also reports whether the direction is negative along that axis.
	 */
	EAxis::Type FindClosestCardinalAxis(const FVector& Dir, bool& bOutNegative)
	{
		const float AbsX = FMath::Abs(Dir.X);
		const float AbsY = FMath::Abs(Dir.Y);
		const float AbsZ = FMath::Abs(Dir.Z);

		if (AbsX >= AbsY && AbsX >= AbsZ)
		{
			bOutNegative = Dir.X < 0.0f;
			return EAxis::X;
		}
		if (AbsY >= AbsZ)
		{
			bOutNegative = Dir.Y < 0.0f;
			return EAxis::Y;
		}
		bOutNegative = Dir.Z < 0.0f;
		return EAxis::Z;
	}

	/**
	 * Determine which constraint bone is the skeletal child (further from root).
	 * Returns the child bone name and the corresponding constraint frame.
	 * Falls back to ConstraintBone1/Frame1 if relationship can't be determined.
	 */
	void FindChildBoneAndFrame(const FConstraintInstance& CI, const FReferenceSkeleton& RefSkeleton,
		FName& OutChildBone, EConstraintFrame::Type& OutChildFrame)
	{
		OutChildBone = CI.ConstraintBone1;
		OutChildFrame = EConstraintFrame::Frame1;

		const int32 Bone1Idx = RefSkeleton.FindBoneIndex(CI.ConstraintBone1);
		const int32 Bone2Idx = RefSkeleton.FindBoneIndex(CI.ConstraintBone2);

		if (Bone1Idx == INDEX_NONE || Bone2Idx == INDEX_NONE)
		{
			return; // can't determine, keep default
		}

		// Walk up from Bone1 to see if Bone2 is an ancestor → Bone1 is child
		for (int32 Idx = RefSkeleton.GetParentIndex(Bone1Idx); Idx != INDEX_NONE; Idx = RefSkeleton.GetParentIndex(Idx))
		{
			if (Idx == Bone2Idx)
			{
				OutChildBone = CI.ConstraintBone1;
				OutChildFrame = EConstraintFrame::Frame1;
				return;
			}
		}

		// Walk up from Bone2 to see if Bone1 is an ancestor → Bone2 is child
		for (int32 Idx = RefSkeleton.GetParentIndex(Bone2Idx); Idx != INDEX_NONE; Idx = RefSkeleton.GetParentIndex(Idx))
		{
			if (Idx == Bone1Idx)
			{
				OutChildBone = CI.ConstraintBone2;
				OutChildFrame = EConstraintFrame::Frame2;
				return;
			}
		}

		// No ancestor relationship found, keep default
	}

	/**
	 * Analyze a physics constraint to determine joint type, axis, and limits.
	 * Uses the specified constraint reference frame to correctly map constraint
	 * axes to bone-local axes, and prefers Limited DOFs over Free ones.
	 * Returns true if a controllable DOF was found.
	 */
	bool AnalyzeConstraint(const FConstraintInstance& CI, EConstraintFrame::Type ChildFrame,
		FKinematicJointConfig& OutJoint)
	{
		// Get constraint frame orientation in bone-local space for the child bone
		const FTransform RefFrame = CI.GetRefFrame(ChildFrame);
		const FQuat		 FrameQuat = RefFrame.GetRotation();

		// Constraint axis directions in bone-local space:
		// Twist = constraint X, Swing1 = constraint Z, Swing2 = constraint Y
		const FVector TwistDir = FrameQuat.GetAxisX();
		const FVector Swing1Dir = FrameQuat.GetAxisZ();
		const FVector Swing2Dir = FrameQuat.GetAxisY();

		// ── Angular (Revolute) ──────────────────────────────────────
		const EAngularConstraintMotion TwistMotion = CI.GetAngularTwistMotion();
		const EAngularConstraintMotion Swing1Motion = CI.GetAngularSwing1Motion();
		const EAngularConstraintMotion Swing2Motion = CI.GetAngularSwing2Motion();

		auto IsActive = [](EAngularConstraintMotion M) {
			return M == EAngularConstraintMotion::ACM_Free || M == EAngularConstraintMotion::ACM_Limited;
		};
		auto IsLimited = [](EAngularConstraintMotion M) {
			return M == EAngularConstraintMotion::ACM_Limited;
		};

		const bool bTwistActive = IsActive(TwistMotion);
		const bool bSwing1Active = IsActive(Swing1Motion);
		const bool bSwing2Active = IsActive(Swing2Motion);

		if (bTwistActive || bSwing1Active || bSwing2Active)
		{
			OutJoint.JointType = EKinematicJointType::Revolute;

			// Build candidates: each active angular DOF with its constraint-frame direction
			struct AngularCandidate
			{
				EAngularConstraintMotion Motion;
				FVector					 Direction;
				float					 Limit;
			};

			TArray<AngularCandidate, TInlineAllocator<3>> Candidates;
			if (bTwistActive)
			{
				Candidates.Add({ TwistMotion, TwistDir, CI.GetAngularTwistLimit() });
			}
			if (bSwing1Active)
			{
				Candidates.Add({ Swing1Motion, Swing1Dir, CI.GetAngularSwing1Limit() });
			}
			if (bSwing2Active)
			{
				Candidates.Add({ Swing2Motion, Swing2Dir, CI.GetAngularSwing2Limit() });
			}

			// Prefer Limited over Free (Limited = intentionally configured DOF)
			int32 BestIdx = 0;
			for (int32 i = 1; i < Candidates.Num(); ++i)
			{
				if (IsLimited(Candidates[i].Motion) && !IsLimited(Candidates[BestIdx].Motion))
				{
					BestIdx = i;
				}
			}

			const AngularCandidate& Best = Candidates[BestIdx];

			// Map constraint axis direction to nearest bone-local cardinal axis
			bool bNegative = false;
			OutJoint.Axis = FindClosestCardinalAxis(Best.Direction, bNegative);
			OutJoint.bInvertDirection = bNegative;

			if (IsLimited(Best.Motion))
			{
				OutJoint.MinValue = -Best.Limit;
				OutJoint.MaxValue = Best.Limit;
				OutJoint.bEnforceLimits = true;
			}
			else
			{
				OutJoint.MinValue = -180.0f;
				OutJoint.MaxValue = 180.0f;
				OutJoint.bEnforceLimits = false;
			}

			return true;
		}

		// ── Linear (Prismatic) ──────────────────────────────────────
		const ELinearConstraintMotion LinXMotion = CI.GetLinearXMotion();
		const ELinearConstraintMotion LinYMotion = CI.GetLinearYMotion();
		const ELinearConstraintMotion LinZMotion = CI.GetLinearZMotion();

		auto IsLinActive = [](ELinearConstraintMotion M) {
			return M == ELinearConstraintMotion::LCM_Free || M == ELinearConstraintMotion::LCM_Limited;
		};
		auto IsLinLimited = [](ELinearConstraintMotion M) {
			return M == ELinearConstraintMotion::LCM_Limited;
		};

		struct LinearCandidate
		{
			ELinearConstraintMotion Motion;
			FVector					Direction;
		};

		TArray<LinearCandidate, TInlineAllocator<3>> LinCandidates;
		// Linear X/Y/Z in constraint frame map to constraint X/Y/Z axes
		if (IsLinActive(LinXMotion))
		{
			LinCandidates.Add({ LinXMotion, FrameQuat.GetAxisX() });
		}
		if (IsLinActive(LinYMotion))
		{
			LinCandidates.Add({ LinYMotion, FrameQuat.GetAxisY() });
		}
		if (IsLinActive(LinZMotion))
		{
			LinCandidates.Add({ LinZMotion, FrameQuat.GetAxisZ() });
		}

		if (LinCandidates.Num() == 0)
		{
			return false; // everything locked
		}

		// Prefer Limited over Free
		int32 BestLinIdx = 0;
		for (int32 i = 1; i < LinCandidates.Num(); ++i)
		{
			if (IsLinLimited(LinCandidates[i].Motion) && !IsLinLimited(LinCandidates[BestLinIdx].Motion))
			{
				BestLinIdx = i;
			}
		}

		OutJoint.JointType = EKinematicJointType::Prismatic;

		bool bNegative = false;
		OutJoint.Axis = FindClosestCardinalAxis(LinCandidates[BestLinIdx].Direction, bNegative);
		OutJoint.bInvertDirection = bNegative;

		const float LinearLimit = CI.GetLinearLimit();
		if (LinearLimit > 0.0f)
		{
			OutJoint.MinValue = -LinearLimit;
			OutJoint.MaxValue = LinearLimit;
			OutJoint.bEnforceLimits = true;
		}
		else
		{
			OutJoint.MinValue = -100.0f;
			OutJoint.MaxValue = 100.0f;
			OutJoint.bEnforceLimits = false;
		}

		return true;
	}
} // anonymous namespace

URammsSkeletalPoseComponent::URammsSkeletalPoseComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
}

// ── Lifecycle ───────────────────────────────────────────────────────────

void URammsSkeletalPoseComponent::BeginPlay()
{
	Super::BeginPlay();

	AActor* Owner = GetOwner();
	if (!Owner)
	{
		return;
	}

	CacheAllPoseableMeshes();
	RebuildNameMap();

	if (PoseableMeshes.Num() == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("RammsSkeletalPoseComponent on %s: No UPoseableMeshComponent(s) found."),
			*Owner->GetName());
	}
	else
	{
		if (bDebugLog)
		{
			UE_LOG(LogTemp, Log, TEXT("RammsSkeletalPoseComponent on %s: Found %d poseable mesh(es), %d joints configured"),
				*Owner->GetName(), PoseableMeshes.Num(), Joints.Num());
			for (auto& Pair : PoseableMeshes)
			{
				UE_LOG(LogTemp, Log, TEXT("  Mesh: '%s'"), *Pair.Key.ToString());
			}

			// Log every joint's full configuration
			for (int32 i = 0; i < Joints.Num(); ++i)
			{
				const FKinematicJointConfig& J = Joints[i];
				const TCHAR*				 TypeStr = J.JointType == EKinematicJointType::Revolute ? TEXT("Revolute") : TEXT("Prismatic");
				const TCHAR*				 AxisStr = J.Axis == EAxis::X ? TEXT("X") : (J.Axis == EAxis::Y ? TEXT("Y") : TEXT("Z"));
				UPoseableMeshComponent*		 Mesh = ResolveMeshForJoint(J);
				UE_LOG(LogTemp, Log, TEXT("  Joint[%d] '%s': Bone='%s' Mesh='%s' Type=%s Axis=%s Invert=%d Limits=[%.1f, %.1f] Resolved=%s"),
					i, *J.GetEffectiveName().ToString(),
					*J.BoneName.ToString(), *J.MeshComponentName.ToString(),
					TypeStr, AxisStr, J.bInvertDirection ? 1 : 0,
					J.MinValue, J.MaxValue,
					Mesh ? *Mesh->GetFName().ToString() : TEXT("NULL"));
			}
		}

		// Warn about any joints whose MeshComponentName doesn't match a discovered mesh
		for (const FKinematicJointConfig& Joint : Joints)
		{
			if (!Joint.MeshComponentName.IsNone() && !PoseableMeshes.Contains(Joint.MeshComponentName))
			{
				UE_LOG(LogTemp, Warning, TEXT("RammsSkeletalPoseComponent on %s: Joint '%s' references mesh '%s' "
											  "which was not found. Will fall back to default mesh '%s'."),
					*Owner->GetName(), *Joint.GetEffectiveName().ToString(),
					*Joint.MeshComponentName.ToString(), *DefaultMeshName.ToString());
				break; // only warn once
			}
		}
	}
}

void URammsSkeletalPoseComponent::TickComponent(float DeltaTime, ELevelTick TickType,
	FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (PoseableMeshes.Num() == 0)
	{
		return;
	}

	for (FKinematicJointConfig& Joint : Joints)
	{
		if (Joint.BoneName.IsNone())
		{
			continue;
		}

		const float Target = ClampToLimits(Joint, Joint.TargetValue);

		if (bEnableInterpolation && InterpolationSpeed > 0.0f)
		{
			const float MaxDelta = InterpolationSpeed * DeltaTime;
			const float Delta = Target - Joint.CurrentValue;
			const float ClampedDelta = FMath::Clamp(Delta, -MaxDelta, MaxDelta);
			Joint.CurrentValue += ClampedDelta;
		}
		else
		{
			Joint.CurrentValue = Target;
		}

		ApplyJointToBone(Joint);
	}
}

// ── Set Joint Targets ───────────────────────────────────────────────────

void URammsSkeletalPoseComponent::SetJointTarget(int32 JointIndex, float Value)
{
	if (Joints.IsValidIndex(JointIndex))
	{
		Joints[JointIndex].TargetValue = Value;
	}
	else if (bDebugLog)
	{
		UE_LOG(LogTemp, Warning, TEXT("SetJointTarget: Index %d out of range (0..%d)"),
			JointIndex, Joints.Num() - 1);
	}
}

void URammsSkeletalPoseComponent::SetJointTargetByName(FName JointName, float Value)
{
	const int32 Index = FindJointIndex(JointName);
	if (Index != INDEX_NONE)
	{
		Joints[Index].TargetValue = Value;
	}
	else if (bDebugLog)
	{
		UE_LOG(LogTemp, Warning, TEXT("SetJointTargetByName: Joint '%s' not found"),
			*JointName.ToString());
	}
}

void URammsSkeletalPoseComponent::SetAllJointTargets(const TArray<double>& Values)
{
	const int32 Count = FMath::Min(Values.Num(), Joints.Num());
	for (int32 i = 0; i < Count; ++i)
	{
		Joints[i].TargetValue = static_cast<float>(Values[i]);
	}

	if (bDebugLog && Values.Num() != Joints.Num())
	{
		UE_LOG(LogTemp, Warning, TEXT("SetAllJointTargets: Provided %d values but have %d joints"),
			Values.Num(), Joints.Num());
	}
}

bool URammsSkeletalPoseComponent::SetJointTargetsFromPoseIndex(
	URammsJointPoseAsset* PoseAsset, int32 PoseIndex)
{
	if (!PoseAsset)
	{
		return false;
	}

	TArray<float> Angles;
	if (!PoseAsset->GetPoseByIndex(PoseIndex, Angles))
	{
		return false;
	}

	TArray<double> AnglesD;
	AnglesD.Reserve(Angles.Num());
	for (float A : Angles)
	{
		AnglesD.Add(static_cast<double>(A));
	}
	SetAllJointTargets(AnglesD);
	return true;
}

bool URammsSkeletalPoseComponent::SetJointTargetsFromPoseName(
	URammsJointPoseAsset* PoseAsset, const FString& PoseName)
{
	if (!PoseAsset)
	{
		return false;
	}

	TArray<float> Angles;
	if (!PoseAsset->GetPoseByName(PoseName, Angles))
	{
		return false;
	}

	TArray<double> AnglesD;
	AnglesD.Reserve(Angles.Num());
	for (float A : Angles)
	{
		AnglesD.Add(static_cast<double>(A));
	}
	SetAllJointTargets(AnglesD);
	return true;
}

void URammsSkeletalPoseComponent::SnapToTargets()
{
	for (FKinematicJointConfig& Joint : Joints)
	{
		Joint.CurrentValue = ClampToLimits(Joint, Joint.TargetValue);
	}

	for (const FKinematicJointConfig& Joint : Joints)
	{
		if (!Joint.BoneName.IsNone())
		{
			ApplyJointToBone(Joint);
		}
	}
}

// ── Read Joint State ────────────────────────────────────────────────────

float URammsSkeletalPoseComponent::GetJointValue(int32 JointIndex) const
{
	if (Joints.IsValidIndex(JointIndex))
	{
		return Joints[JointIndex].CurrentValue;
	}
	return 0.0f;
}

float URammsSkeletalPoseComponent::GetJointValueByName(FName JointName) const
{
	const int32 Index = FindJointIndex(JointName);
	return Index != INDEX_NONE ? Joints[Index].CurrentValue : 0.0f;
}

TArray<float> URammsSkeletalPoseComponent::GetAllJointValues() const
{
	TArray<float> Values;
	Values.Reserve(Joints.Num());
	for (const FKinematicJointConfig& Joint : Joints)
	{
		Values.Add(Joint.CurrentValue);
	}
	return Values;
}

float URammsSkeletalPoseComponent::GetJointTarget(int32 JointIndex) const
{
	if (Joints.IsValidIndex(JointIndex))
	{
		return Joints[JointIndex].TargetValue;
	}
	return 0.0f;
}

TArray<float> URammsSkeletalPoseComponent::GetAllJointTargets() const
{
	TArray<float> Values;
	Values.Reserve(Joints.Num());
	for (const FKinematicJointConfig& Joint : Joints)
	{
		Values.Add(Joint.TargetValue);
	}
	return Values;
}

// ── Mesh Discovery ──────────────────────────────────────────────────────

TArray<FName> URammsSkeletalPoseComponent::GetPoseableMeshNames() const
{
	TArray<FName> Names;
	PoseableMeshes.GetKeys(Names);
	return Names;
}

void URammsSkeletalPoseComponent::RefreshPoseableMeshes()
{
	CacheAllPoseableMeshes();
}

// ── Joint Discovery ─────────────────────────────────────────────────────

void URammsSkeletalPoseComponent::AutoPopulateFromSkeleton()
{
	CacheAllPoseableMeshes();

	if (PoseableMeshes.Num() == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("AutoPopulateFromSkeleton: No poseable meshes found on actor"));
		return;
	}

	Joints.Empty();

	for (auto& Pair : PoseableMeshes)
	{
		const FName				MeshName = Pair.Key;
		UPoseableMeshComponent* Mesh = Pair.Value;

		if (!Mesh || !Mesh->GetSkinnedAsset())
		{
			continue;
		}

		const USkeletalMesh* SkelMesh = Cast<USkeletalMesh>(Mesh->GetSkinnedAsset());
		if (!SkelMesh)
		{
			continue;
		}

		// Try physics asset first for accurate axis/limit detection
		// Check component override (PhysicsAssetOverride) first, then skeletal mesh asset
		const UPhysicsAsset* PhysAsset = Mesh->GetPhysicsAsset();
		if (!PhysAsset)
		{
			PhysAsset = SkelMesh->GetPhysicsAsset();
		}

		int32 PhysJoints = 0;
		if (PhysAsset)
		{
			const FReferenceSkeleton& RefSkeleton = SkelMesh->GetRefSkeleton();

			for (const UPhysicsConstraintTemplate* Template : PhysAsset->ConstraintSetup)
			{
				if (!Template)
				{
					continue;
				}

				const FConstraintInstance& CI = Template->DefaultInstance;

				// Determine which constraint bone is the child (the one we pose)
				FName				   ChildBone;
				EConstraintFrame::Type ChildFrame;
				FindChildBoneAndFrame(CI, RefSkeleton, ChildBone, ChildFrame);

				FKinematicJointConfig Joint;
				if (!AnalyzeConstraint(CI, ChildFrame, Joint))
				{
					continue;
				}

				Joint.MeshComponentName = MeshName;
				Joint.BoneName = ChildBone;
				Joint.JointName = CI.JointName;
				Joints.Add(Joint);
				++PhysJoints;
			}
		}

		if (PhysJoints > 0)
		{
			if (bDebugLog)
			{
				UE_LOG(LogTemp, Log, TEXT("AutoPopulateFromSkeleton: Added %d joints from physics asset on mesh '%s'"),
					PhysJoints, *MeshName.ToString());
			}
		}
		else
		{
			// Fallback: create one revolute joint per non-root bone
			const FReferenceSkeleton& RefSkeleton = SkelMesh->GetRefSkeleton();
			const int32				  NumBones = RefSkeleton.GetNum();

			for (int32 i = 1; i < NumBones; ++i)
			{
				FKinematicJointConfig Joint;
				Joint.MeshComponentName = MeshName;
				Joint.BoneName = RefSkeleton.GetBoneName(i);
				Joint.JointName = Joint.BoneName;
				Joint.JointType = EKinematicJointType::Revolute;
				Joint.Axis = EAxis::X;
				Joints.Add(Joint);
			}

			if (bDebugLog)
			{
				UE_LOG(LogTemp, Log, TEXT("AutoPopulateFromSkeleton: Added %d joints from skeleton (no physics asset) on mesh '%s'"),
					NumBones - 1, *MeshName.ToString());
			}
		}
	}

	RebuildNameMap();

	UE_LOG(LogTemp, Log, TEXT("AutoPopulateFromSkeleton: Created %d total joints from %d mesh(es)"),
		Joints.Num(), PoseableMeshes.Num());
}

// ── Pose Capture ────────────────────────────────────────────────────────

void URammsSkeletalPoseComponent::CaptureCurrentPose(
	URammsJointPoseAsset* PoseAsset, const FString& PoseName)
{
	if (!PoseAsset)
	{
		return;
	}

	TArray<float> Values = GetAllJointValues();

	// Check if a pose with this name already exists
	for (FRammsJointPose& Pose : PoseAsset->Poses)
	{
		if (Pose.PoseName == PoseName)
		{
			Pose.JointAngles = Values;
			PoseAsset->MarkPackageDirty();
			return;
		}
	}

	// Add new pose
	FRammsJointPose NewPose;
	NewPose.PoseName = PoseName;
	NewPose.JointAngles = Values;
	PoseAsset->Poses.Add(NewPose);
	PoseAsset->MarkPackageDirty();
}

// ── Private ─────────────────────────────────────────────────────────────

void URammsSkeletalPoseComponent::CacheAllPoseableMeshes()
{
	PoseableMeshes.Empty();
	DefaultMeshName = NAME_None;

	AActor* Owner = GetOwner();
	if (!Owner)
	{
		return;
	}

	TArray<UPoseableMeshComponent*> Found;
	Owner->GetComponents<UPoseableMeshComponent>(Found);

	for (UPoseableMeshComponent* Mesh : Found)
	{
		if (Mesh)
		{
			const FName Name = Mesh->GetFName();
			PoseableMeshes.Add(Name, Mesh);

			if (DefaultMeshName.IsNone())
			{
				DefaultMeshName = Name;
			}
		}
	}

	if (bDebugLog)
	{
		UE_LOG(LogTemp, Log, TEXT("CacheAllPoseableMeshes: Found %d mesh(es), default='%s'"),
			PoseableMeshes.Num(), *DefaultMeshName.ToString());
	}
}

void URammsSkeletalPoseComponent::RebuildNameMap()
{
	JointNameMap.Empty(Joints.Num());
	for (int32 i = 0; i < Joints.Num(); ++i)
	{
		const FName Name = Joints[i].GetEffectiveName();
		if (Name.IsNone())
		{
			continue;
		}

		if (const int32* Existing = JointNameMap.Find(Name))
		{
			// Duplicate name — qualify with MeshComponentName to disambiguate
			const FName QualifiedName = *FString::Printf(TEXT("%s::%s"),
				*Joints[i].MeshComponentName.ToString(), *Name.ToString());
			JointNameMap.Add(QualifiedName, i);

			// Also qualify the earlier entry if it hasn't been already
			const FKinematicJointConfig& Earlier = Joints[*Existing];
			const FName					 EarlierQualified = *FString::Printf(TEXT("%s::%s"),
								 *Earlier.MeshComponentName.ToString(), *Name.ToString());
			if (!JointNameMap.Contains(EarlierQualified))
			{
				JointNameMap.Add(EarlierQualified, *Existing);
			}

			if (bDebugLog)
			{
				UE_LOG(LogTemp, Warning, TEXT("RebuildNameMap: Duplicate joint name '%s' at indices %d and %d. "
											  "Use qualified names '%s' and '%s' for by-name access."),
					*Name.ToString(), *Existing, i,
					*EarlierQualified.ToString(), *QualifiedName.ToString());
			}
		}
		else
		{
			JointNameMap.Add(Name, i);
		}
	}
}

UPoseableMeshComponent* URammsSkeletalPoseComponent::ResolveMeshForJoint(
	const FKinematicJointConfig& Joint) const
{
	const FName TargetName = Joint.MeshComponentName.IsNone() ? DefaultMeshName : Joint.MeshComponentName;

	if (const TObjectPtr<UPoseableMeshComponent>* Found = PoseableMeshes.Find(TargetName))
	{
		return Found->Get();
	}

	// Fallback: if specific mesh name wasn't found, try the default mesh
	if (!Joint.MeshComponentName.IsNone() && !DefaultMeshName.IsNone())
	{
		if (const TObjectPtr<UPoseableMeshComponent>* Fallback = PoseableMeshes.Find(DefaultMeshName))
		{
			return Fallback->Get();
		}
	}

	return nullptr;
}

void URammsSkeletalPoseComponent::ApplyJointToBone(const FKinematicJointConfig& Joint) const
{
	UPoseableMeshComponent* Mesh = ResolveMeshForJoint(Joint);
	if (!Mesh)
	{
		return;
	}

	// Reset to bind pose first so we have a clean baseline each frame.
	// Without this, component-space transforms can accumulate incorrectly,
	// especially for bones on child poseable meshes in hierarchies.
	Mesh->ResetBoneTransformByName(Joint.BoneName);

	const float SignedValue = Joint.bInvertDirection ? -Joint.CurrentValue : Joint.CurrentValue;
	const float FinalValue = SignedValue + Joint.Offset;

	if (Joint.JointType == EKinematicJointType::Revolute)
	{
		// Read the bind pose rotation (just restored by Reset above)
		const FQuat BindQuat = Mesh->GetBoneRotationByName(
									   Joint.BoneName, EBoneSpaces::ComponentSpace)
								   .Quaternion();

		// Build the delta rotation around the specified axis in bone-local frame
		FRotator DeltaRotation = FRotator::ZeroRotator;
		switch (Joint.Axis)
		{
			case EAxis::X:
				DeltaRotation.Roll = FinalValue;
				break;
			case EAxis::Y:
				DeltaRotation.Pitch = FinalValue;
				break;
			case EAxis::Z:
				DeltaRotation.Yaw = FinalValue;
				break;
			default:
				break;
		}

		// Compose: bind pose * delta → applies delta in the bone's local frame
		// When FinalValue=0, DeltaQuat is identity → bone stays at bind pose
		const FQuat FinalQuat = BindQuat * DeltaRotation.Quaternion();
		Mesh->SetBoneRotationByName(Joint.BoneName, FinalQuat.Rotator(), EBoneSpaces::ComponentSpace);
	}
	else // Prismatic
	{
		// Read the bind pose transform
		const FVector BindLocation = Mesh->GetBoneLocationByName(
			Joint.BoneName, EBoneSpaces::ComponentSpace);
		const FQuat BindQuat = Mesh->GetBoneRotationByName(
									   Joint.BoneName, EBoneSpaces::ComponentSpace)
								   .Quaternion();

		// Build delta in bone-local frame
		FVector LocalDelta = FVector::ZeroVector;
		switch (Joint.Axis)
		{
			case EAxis::X:
				LocalDelta.X = FinalValue;
				break;
			case EAxis::Y:
				LocalDelta.Y = FinalValue;
				break;
			case EAxis::Z:
				LocalDelta.Z = FinalValue;
				break;
			default:
				break;
		}

		// Rotate delta into component space, then add to bind location
		const FVector ComponentDelta = BindQuat.RotateVector(LocalDelta);
		Mesh->SetBoneLocationByName(Joint.BoneName, BindLocation + ComponentDelta,
			EBoneSpaces::ComponentSpace);
	}
}

float URammsSkeletalPoseComponent::ClampToLimits(const FKinematicJointConfig& Joint, float Value)
{
	if (Joint.bEnforceLimits)
	{
		return FMath::Clamp(Value, Joint.MinValue, Joint.MaxValue);
	}
	return Value;
}

int32 URammsSkeletalPoseComponent::FindJointIndex(FName JointName) const
{
	const int32* FoundIndex = JointNameMap.Find(JointName);
	return FoundIndex ? *FoundIndex : INDEX_NONE;
}
