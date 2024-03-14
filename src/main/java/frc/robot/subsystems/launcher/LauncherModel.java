// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.LiftConstants;

/**
 * A class for getting a LaunchTarget using a physics-based model
 */
public class LauncherModel {
    // Universal constants
    private static final double kGravity = -9.81;

    // Robot dimensions and offsets
    private static final double kDistanceFromLimelightToBotCenterMeters = 0.177306;
    private static final double kDistanceFromBotCenterToPivotMeters = 0.299;
    private static final double kFrontLimelightHeightMeters = 0.452;
    private static final double kFrontLimelightOffsetAngleDegrees = 24.89;
    private static final double kLaunchAxisOffsetMeters = 0.065;
    private static final double kLaunchExitOffsetMeters = 0.239;
    private static final double kLauncherPivotHeightMeters = 0.277;
    private static final Rotation2d kLiftAngle = Rotation2d.fromDegrees(10); 

    // Speaker and April tag dimensions
    private static final double kAprilTagHeightMeters = 1.469;
    private static final double kAprilTagLimelightHeightDifferenceMeters = kAprilTagHeightMeters - kFrontLimelightHeightMeters;
    private static final double kDistanceOffsetFromSpeakerTagToSpeakerOpeningMeters = 0.2664;

    // Launcher speed linear calculation numbers
    private static final double kMinLauncherSpeedMps = 18.8;
    private static final double kMaxLauncherSpeedMps = 32.0;
    private static final double kMaxDistanceForMinSpeedMeters = 3.0;
    private static final double kMinDistanceForMaxSpeedMeters = 6.0;
    private static final double kLauncherSpeedSlope = (kMaxLauncherSpeedMps - kMinLauncherSpeedMps) / (kMinDistanceForMaxSpeedMeters - kMaxDistanceForMinSpeedMeters);

    // Flywheel to motor speeds constants
    private static final double kFlywheelDiameter = 0.1016;
    private static final double kFlywheelGearRatio = 26.0 / 22.0;

    /**
     * An enum for specifying if we're aiming for a speaker or the floor
     */
    public enum LauncherHeightTarget {
        Floor(0.0),
        Speaker(2.078);

        public double heightMeters;

        LauncherHeightTarget (double heightMeters) {
            this.heightMeters = heightMeters;
        }
    }

    public static LauncherTarget getLauncherTarget(LauncherHeightTarget heightTarget, double liftHeightMeters, double distanceToTargetFromBotCenterMeters, Rotation2d currentTiltAngle) {
        // Calculate the distance from the launch point to the target
        double adjustedDistanceMeters = distanceToTargetFromBotCenterMeters + kDistanceFromBotCenterToPivotMeters - getLauncherDistanceToPivotMeters(currentTiltAngle) + getLiftDistanceOffsetMeters(liftHeightMeters);

        // Adjust the height to at least be above the min height calculated for the current distance
        double adjustedLiftHeightMeters = Math.max(liftHeightMeters, getMinLiftHeightMetersForDistanceMeters(adjustedDistanceMeters));

        // Calculate the initial velocity target for the flywheels
        double initialVelocityMPS = interpolateInitialVelocityMps(adjustedDistanceMeters);

        // Calculate the height difference from the launch point to the target (positive for speaker, negative for floor)
        double launchHeightDifferenceMeters = heightTarget.heightMeters - kLauncherPivotHeightMeters - getLiftHeightOffsetMeters(adjustedLiftHeightMeters) - getLauncherHeightAbovePivotMeters(currentTiltAngle);

        // Calculate the desired launcher angle from the calculated values
        // Using the formula Dan gave us here: https://docs.google.com/spreadsheets/d/1uqDhTsbwMxrkkyMMzWEmLbcNDx1oy95dRZYe3OpYG2s/edit?usp=sharing
        double vSquared = Math.pow(initialVelocityMPS, 2);
        double sqrtExpression = Math.sqrt(Math.pow(initialVelocityMPS, 4) - kGravity * (kGravity*Math.pow(adjustedDistanceMeters, 2) + 2 * -launchHeightDifferenceMeters * Math.pow(initialVelocityMPS, 2)));
        double gx =  (kGravity * adjustedDistanceMeters);
        double expression = Math.abs((vSquared - sqrtExpression) / gx);
        double thetaRads = Math.atan(expression);
        double thetaDegrees = Math.toDegrees(thetaRads);

        // Convert flywheel speed (mps) to the RPM we need for the launcher
        double launcherRPM = mpsToLauncherRPM(initialVelocityMPS);

        // Return the launcher target
        return new LauncherTarget(adjustedDistanceMeters, 0, launcherRPM, 0, thetaDegrees, adjustedLiftHeightMeters);
    }

    /**
     * Calculate the flywheel speed (mps) for the given distance to the goal
     * @param distance the distance to the goal
     */
    public static double interpolateInitialVelocityMps(double distance) {
        double interpolatedSpeed = (kLauncherSpeedSlope * (distance - kMaxDistanceForMinSpeedMeters)) + kMinLauncherSpeedMps;
        return MathUtil.clamp(interpolatedSpeed, kMinLauncherSpeedMps, kMaxLauncherSpeedMps);
    }

    /**
     * Converts from the physical flywheel speed target (mps) to the launcher speed (RPM)
     * @param mps the target speed in meters per second
     */
    public static double mpsToLauncherRPM(double mps) {
        double metersPerMinute = mps * 60;
        double wheelRPM = metersPerMinute / (Math.PI * kFlywheelDiameter);
        double drivenRPM = wheelRPM / kFlywheelGearRatio;
        return drivenRPM;
    }

    /**
     * Gets the launcher height offset based on the current tilt angle
     * @param currentTiltAngle the current launcher tilt angle
     */
    public static double getLauncherHeightAbovePivotMeters(Rotation2d currentTiltAngle) {
        double tiltAngleRadians = currentTiltAngle.getRadians();

        // =(COS(RADIANS(I30))*J9) + (sin(RADIANS(I30))*J10)
        double launcherHeightAbovePivotMeters = (Math.cos(tiltAngleRadians) * kLaunchAxisOffsetMeters) + (Math.sin(tiltAngleRadians) * kLaunchExitOffsetMeters);

        return launcherHeightAbovePivotMeters;
    }

    /**
     * Gets the launcher distance offset based on the current tilt angle
     * @param currentTiltAngle the current launcher tilt angle
     */
    public static double getLauncherDistanceToPivotMeters(Rotation2d currentTiltAngle) {
        double tiltAngleRadians = currentTiltAngle.getRadians();

        //-(sin(RADIANS(I30))*J9) + (COS(RADIANS(I30))*J10)
        double launcherHeightAbovePivotMeters = -(Math.sin(tiltAngleRadians) * kLaunchAxisOffsetMeters) + (Math.cos(tiltAngleRadians) * kLaunchExitOffsetMeters);

        return launcherHeightAbovePivotMeters;
    }

    /**
     * Gets the lift's vertical height (the lift measures it's height at a 10 deg slant)
     * @param currentLiftHeight the current height (at a slant) of the lift
     */
    public static double getLiftHeightOffsetMeters(double currentLiftHeight) {
        // COS(RADIANS(10))*B21
        return Math.cos(kLiftAngle.getRadians()) * currentLiftHeight;
    }

    /**
     * Gets the lift's horizontal distance (the lift measures it's height at a 10 deg slant)
     * @param currentLiftHeight the current height (at a slant) of the lift
     */
    public static double getLiftDistanceOffsetMeters(double currentLiftHeight) {
        // SIN(RADIANS(10))*B21
        return Math.sin(kLiftAngle.getRadians()) * currentLiftHeight;
    }

    /**
     * Converts from a TY value to distance meters from the camera to the speaker opening
     * @param ty the degrees the april tag target is above the camera's center line
     */
    public static double speakerAprilTagTyToBotCenterDistanceMeters(double ty) {
        double distanceMeters = kAprilTagLimelightHeightDifferenceMeters / Math.tan(Math.toRadians(kFrontLimelightOffsetAngleDegrees + ty));
        return distanceMeters + kDistanceFromLimelightToBotCenterMeters - kDistanceOffsetFromSpeakerTagToSpeakerOpeningMeters;
    }

    /**
     * Gets the min lift height for a given distance to the goal
     * @param distanceMeters
     */
    public static double getMinLiftHeightMetersForDistanceMeters(double distanceMeters) {
        //-0.191 + 0.117x + -0.0141x^2 + 6.06E-04x^3
        double calculatedHeight = -0.191 + (0.117 * distanceMeters) + (-0.0141 * Math.pow(distanceMeters, 2)) + (0.000606 * Math.pow(distanceMeters, 3));
        return MathUtil.clamp(calculatedHeight, LiftConstants.MinHeightMeters, LiftConstants.MaxHeightMeters);
    }
}
