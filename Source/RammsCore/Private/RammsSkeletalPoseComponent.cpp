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
	 * Analyze a physics constraint to determine joint type, axis, and limits.
	 * Checks angular axes first (revolute), then linear axes (prismatic).
	 * Returns true if a controllable axis was found.
	 */
	bool AnalyzeConstraint(const FConstraintInstance& CI, FKinematicJointConfig& OutJoint)
	{
		// ── Angular (Revolute) ──────────────────────────────────────
		const EAngularConstraintMotion TwistMotion = CI.GetAngularTwistMotion();
		const EAngularConstraintMotion Swing1Motion = CI.GetAngularSwing1Motion();
		const EAngularConstraintMotion Swing2Motion = CI.GetAngularSwing2Motion();

		auto IsAngularFreeOrLimited = [](EAngularConstraintMotion M) {
			return M == EAngularConstraintMotion::ACM_Free || M == EAngularConstraintMotion::ACM_Limited;
		};

		const bool bTwist = IsAngularFreeOrLimited(TwistMotion);
		const bool bSwing1 = IsAngularFreeOrLimited(Swing1Motion);
		const bool bSwing2 = IsAngularFreeOrLimited(Swing2Motion);

		if (bTwist || bSwing1 || bSwing2)
		{
			OutJoint.JointType = EKinematicJointType::Revolute;

			if (bTwist)
			{
				OutJoint.Axis = EAxis::X;
				if (TwistMotion == EAngularConstraintMotion::ACM_Limited)
				{
					const float Limit = CI.GetAngularTwistLimit();
					OutJoint.MinValue = -Limit;
					OutJoint.MaxValue = Limit;
					OutJoint.bEnforceLimits = true;
				}
				else
				{
					OutJoint.MinValue = -180.0f;
					OutJoint.MaxValue = 180.0f;
					OutJoint.bEnforceLimits = false;
				}
			}
			else if (bSwing1 && bSwing2)
			{
				const float S1Limit = CI.GetAngularSwing1Limit();
				const float S2Limit = CI.GetAngularSwing2Limit();
				if (S2Limit > S1Limit)
				{
					OutJoint.Axis = EAxis::Y;
					OutJoint.MinValue = -S2Limit;
					OutJoint.MaxValue = S2Limit;
				}
				else
				{
					OutJoint.Axis = EAxis::Z;
					OutJoint.MinValue = -S1Limit;
					OutJoint.MaxValue = S1Limit;
				}
				OutJoint.bEnforceLimits = true;
			}
			else if (bSwing1)
			{
				OutJoint.Axis = EAxis::Z;
				if (Swing1Motion == EAngularConstraintMotion::ACM_Limited)
				{
					const float Limit = CI.GetAngularSwing1Limit();
					OutJoint.MinValue = -Limit;
					OutJoint.MaxValue = Limit;
					OutJoint.bEnforceLimits = true;
				}
				else
				{
					OutJoint.MinValue = -180.0f;
					OutJoint.MaxValue = 180.0f;
					OutJoint.bEnforceLimits = false;
				}
			}
			else
			{
				OutJoint.Axis = EAxis::Y;
				if (Swing2Motion == EAngularConstraintMotion::ACM_Limited)
				{
					const float Limit = CI.GetAngularSwing2Limit();
					OutJoint.MinValue = -Limit;
					OutJoint.MaxValue = Limit;
					OutJoint.bEnforceLimits = true;
				}
				else
				{
					OutJoint.MinValue = -180.0f;
					OutJoint.MaxValue = 180.0f;
					OutJoint.bEnforceLimits = false;
				}
			}

			return true;
		}

		// ── Linear (Prismatic) ──────────────────────────────────────
		const ELinearConstraintMotion LinXMotion = CI.GetLinearXMotion();
		const ELinearConstraintMotion LinYMotion = CI.GetLinearYMotion();
		const ELinearConstraintMotion LinZMotion = CI.GetLinearZMotion();

		auto IsLinearFreeOrLimited = [](ELinearConstraintMotion M) {
			return M == ELinearConstraintMotion::LCM_Free || M == ELinearConstraintMotion::LCM_Limited;
		};

		const bool bLinX = IsLinearFreeOrLimited(LinXMotion);
		const bool bLinY = IsLinearFreeOrLimited(LinYMotion);
		const bool bLinZ = IsLinearFreeOrLimited(LinZMotion);

		if (!bLinX && !bLinY && !bLinZ)
		{
			return false; // everything locked
		}

		OutJoint.JointType = EKinematicJointType::Prismatic;
		const float LinearLimit = CI.GetLinearLimit();

		// Pick the first free/limited linear axis
		if (bLinX)
		{
			OutJoint.Axis = EAxis::X;
		}
		else if (bLinY)
		{
			OutJoint.Axis = EAxis::Y;
		}
		else
		{
			OutJoint.Axis = EAxis::Z;
		}

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

	/**
	 * Build joint configs from a skeletal mesh's physics asset constraints.
	 * Returns the number of joints created. Joints are appended to OutJoints.
	 */
	int32 BuildJointsFromPhysicsAsset(const USkeletalMesh* SkelMesh, FName MeshComponentName,
		TArray<FKinematicJointConfig>& OutJoints)
	{
		if (!SkelMesh)
		{
			return 0;
		}

		const UPhysicsAsset* PhysAsset = SkelMesh->GetPhysicsAsset();
		if (!PhysAsset)
		{
			return 0;
		}

		int32 Count = 0;
		for (const UPhysicsConstraintTemplate* Template : PhysAsset->ConstraintSetup)
		{
			if (!Template)
			{
				continue;
			}

			const FConstraintInstance& CI = Template->DefaultInstance;

			FKinematicJointConfig Joint;
			if (!AnalyzeConstraint(CI, Joint))
			{
				continue; // fully locked
			}

			Joint.MeshComponentName = MeshComponentName;
			Joint.BoneName = CI.ConstraintBone1;
			Joint.JointName = CI.JointName;
			OutJoints.Add(Joint);
			++Count;
		}

		return Count;
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

void URammsSkeletalPoseComponent::SetAllJointTargets(const TArray<float>& Values)
{
	const int32 Count = FMath::Min(Values.Num(), Joints.Num());
	for (int32 i = 0; i < Count; ++i)
	{
		Joints[i].TargetValue = Values[i];
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

	SetAllJointTargets(Angles);
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

	SetAllJointTargets(Angles);
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
		const FName MeshName = Pair.Key;
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
		const int32 PhysJoints = BuildJointsFromPhysicsAsset(SkelMesh, MeshName, Joints);

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
			const int32 NumBones = RefSkeleton.GetNum();

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
		if (!Name.IsNone())
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
			Joint.BoneName, EBoneSpaces::ComponentSpace).Quaternion();

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
			Joint.BoneName, EBoneSpaces::ComponentSpace).Quaternion();

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
