// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsDifferentialDriveLibrary.h"
#include "Math/UnrealMathUtility.h"

FDifferentialDriveCommand URammsDifferentialDriveLibrary::JoystickToDifferentialDrive(
	FVector2D JoystickInput,
	float MaxValue,
	float DeadZone)
{
	// Apply dead zone
	FVector2D Input = ApplyDeadZone2D(JoystickInput, DeadZone);

	// Standard differential drive mixing
	// Y axis = forward/backward (throttle)
	// X axis = left/right (steering)
	float Throttle = FMath::Clamp(Input.Y, -1.0f, 1.0f);
	float Steering = FMath::Clamp(Input.X, -1.0f, 1.0f);

	// Calculate left and right values
	// When turning left (negative steering), left wheel slows/reverses, right speeds up
	// When turning right (positive steering), right wheel slows/reverses, left speeds up
	float Left = Throttle - Steering;
	float Right = Throttle + Steering;

	// Normalize if either value exceeds 1.0 to maintain turn rate
	float MaxMagnitude = FMath::Max(FMath::Abs(Left), FMath::Abs(Right));
	if (MaxMagnitude > 1.0f)
	{
		Left /= MaxMagnitude;
		Right /= MaxMagnitude;
	}

	// Scale to max value
	return FDifferentialDriveCommand(Left * MaxValue, Right * MaxValue);
}

FDifferentialDriveCommand URammsDifferentialDriveLibrary::CalculateWheelVelocities(
	float LinearVelocity,
	float AngularVelocity,
	float TrackWidth,
	float WheelRadius)
{
	// Convert angular velocity from degrees/s to rad/s
	float AngularVelRad = FMath::DegreesToRadians(AngularVelocity);

	// Differential drive kinematics
	// V_left = (2 * V - omega * L) / (2 * R)
	// V_right = (2 * V + omega * L) / (2 * R)
	// Where: V = linear velocity, omega = angular velocity, L = track width, R = wheel radius

	float HalfTrack = TrackWidth * 0.5f;
	
	// Calculate linear velocity at each wheel
	float LeftLinearVel = LinearVelocity - (AngularVelRad * HalfTrack);
	float RightLinearVel = LinearVelocity + (AngularVelRad * HalfTrack);

	// Convert to angular velocity (rad/s)
	float LeftAngularVel = LeftLinearVel / WheelRadius;
	float RightAngularVel = RightLinearVel / WheelRadius;

	return FDifferentialDriveCommand(LeftAngularVel, RightAngularVel);
}

void URammsDifferentialDriveLibrary::CalculateChassisVelocity(
	float LeftWheelVelocity,
	float RightWheelVelocity,
	float TrackWidth,
	float WheelRadius,
	float& OutLinearVelocity,
	float& OutAngularVelocity)
{
	// Inverse differential drive kinematics
	// V = R * (omega_left + omega_right) / 2
	// omega = R * (omega_right - omega_left) / L
	
	// Linear velocity (cm/s)
	OutLinearVelocity = WheelRadius * (LeftWheelVelocity + RightWheelVelocity) * 0.5f;

	// Angular velocity (rad/s)
	float AngularVelRad = WheelRadius * (RightWheelVelocity - LeftWheelVelocity) / TrackWidth;

	// Convert to degrees/s
	OutAngularVelocity = FMath::RadiansToDegrees(AngularVelRad);
}

float URammsDifferentialDriveLibrary::EvaluateMotorTorque(
	float CurrentRPM,
	const FMotorParameters& MotorParams,
	float RequestedTorque)
{
	float AbsRPM = FMath::Abs(CurrentRPM);
	float Sign = FMath::Sign(RequestedTorque);
	float AbsTorque = FMath::Abs(RequestedTorque);
	
	// Check if motor is at or beyond max RPM
	if (AbsRPM >= MotorParams.MaxRPM)
	{
		// Motor has reached max speed
		// Only allow torque in the opposite direction (braking)
		float RPMSign = FMath::Sign(CurrentRPM);
		if (Sign == RPMSign)
		{
			// Trying to accelerate further - no torque available
			return 0.0f;
		}
		else
		{
			// Braking/reversing - allow full torque
			return AbsTorque * Sign;
		}
	}

	// Calculate available torque based on motor speed
	// Linear interpolation between max torque at 0 RPM and reduced torque at max RPM
	float SpeedRatio = AbsRPM / MotorParams.MaxRPM;
	float AvailableTorque = FMath::Lerp(
		MotorParams.MaxTorque,
		MotorParams.MaxTorque * MotorParams.TorqueAtMaxRPM,
		SpeedRatio);

	// Clamp requested torque to available torque
	AbsTorque = FMath::Min(AbsTorque, AvailableTorque);

	return AbsTorque * Sign;
}

float URammsDifferentialDriveLibrary::RadPerSecToRPM(float RadPerSec)
{
	// RPM = (rad/s) * (60 / 2*PI)
	return RadPerSec * (60.0f / (2.0f * PI));
}

float URammsDifferentialDriveLibrary::RPMToRadPerSec(float RPM)
{
	// rad/s = RPM * (2*PI / 60)
	return RPM * (2.0f * PI / 60.0f);
}

FOdometryData URammsDifferentialDriveLibrary::UpdateOdometry(
	const FOdometryData& CurrentOdometry,
	float LeftWheelDelta,
	float RightWheelDelta,
	float TrackWidth,
	float WheelRadius,
	float DeltaTime)
{
	FOdometryData NewOdometry = CurrentOdometry;

	// Calculate distance traveled by each wheel
	float LeftDistance = LeftWheelDelta * WheelRadius;
	float RightDistance = RightWheelDelta * WheelRadius;

	// Update cumulative wheel distances
	NewOdometry.LeftWheelDistance += LeftDistance;
	NewOdometry.RightWheelDistance += RightDistance;

	// Calculate center distance and angle change
	float CenterDistance = (LeftDistance + RightDistance) * 0.5f;
	float AngleChange = (RightDistance - LeftDistance) / TrackWidth; // in radians

	// Update total distance
	NewOdometry.TotalDistance += FMath::Abs(CenterDistance);

	// Get current yaw in radians
	float CurrentYaw = FMath::DegreesToRadians(CurrentOdometry.Orientation.Yaw);

	// Calculate position change in world frame
	// Use midpoint of rotation for better accuracy
	float MidYaw = CurrentYaw + (AngleChange * 0.5f);
	float DeltaX = CenterDistance * FMath::Cos(MidYaw);
	float DeltaY = CenterDistance * FMath::Sin(MidYaw);

	// Update position
	NewOdometry.Position.X += DeltaX;
	NewOdometry.Position.Y += DeltaY;

	// Update orientation
	float NewYaw = CurrentYaw + AngleChange;
	NewOdometry.Orientation.Yaw = FMath::RadiansToDegrees(NewYaw);

	// Normalize yaw to -180 to 180
	NewOdometry.Orientation.Yaw = FMath::Fmod(NewOdometry.Orientation.Yaw + 180.0f, 360.0f) - 180.0f;

	// Calculate velocities
	if (DeltaTime > SMALL_NUMBER)
	{
		// Linear velocity
		float VelocityMagnitude = CenterDistance / DeltaTime;
		NewOdometry.LinearVelocity.X = VelocityMagnitude * FMath::Cos(MidYaw);
		NewOdometry.LinearVelocity.Y = VelocityMagnitude * FMath::Sin(MidYaw);
		NewOdometry.LinearVelocity.Z = 0.0f;

		// Angular velocity (around Z axis)
		NewOdometry.AngularVelocity.X = 0.0f;
		NewOdometry.AngularVelocity.Y = 0.0f;
		NewOdometry.AngularVelocity.Z = FMath::RadiansToDegrees(AngleChange / DeltaTime);
	}

	return NewOdometry;
}

FOdometryData URammsDifferentialDriveLibrary::ResetOdometry(FVector Position, FRotator Orientation)
{
	FOdometryData NewOdometry;
	NewOdometry.Position = Position;
	NewOdometry.Orientation = Orientation;
	return NewOdometry;
}

float URammsDifferentialDriveLibrary::CalculateSlipRatio(float DesiredVelocity, float ActualVelocity)
{
	float AbsDesired = FMath::Abs(DesiredVelocity);
	
	// Avoid division by zero
	if (AbsDesired < SMALL_NUMBER)
	{
		return 0.0f;
	}

	// Slip ratio = (desired - actual) / desired
	float Slip = (AbsDesired - FMath::Abs(ActualVelocity)) / AbsDesired;
	return FMath::Clamp(Slip, 0.0f, 1.0f);
}

float URammsDifferentialDriveLibrary::ApplyDeadZone(float Value, float DeadZone)
{
	float AbsValue = FMath::Abs(Value);
	if (AbsValue < DeadZone)
	{
		return 0.0f;
	}

	// Scale the remaining range to 0-1
	float Sign = FMath::Sign(Value);
	float Scaled = (AbsValue - DeadZone) / (1.0f - DeadZone);
	return Scaled * Sign;
}

FVector2D URammsDifferentialDriveLibrary::ApplyDeadZone2D(FVector2D Value, float DeadZone)
{
	// Apply radial dead zone
	float Magnitude = Value.Size();
	if (Magnitude < DeadZone)
	{
		return FVector2D::ZeroVector;
	}

	// Scale the remaining range
	float Scaled = (Magnitude - DeadZone) / (1.0f - DeadZone);
	return Value.GetSafeNormal() * Scaled;
}
