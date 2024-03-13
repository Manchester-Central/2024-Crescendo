// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.LiftConstants;

/** Add your docs here. */
public class LauncherModel {
    private static final double kGravity = -9.81;
    private static final double kLauncherPivotHeightMeters = 0.277;
    private static final double kMinLauncherSpeedMps = 18.8;
    private static final double kMaxLauncherSpeedMps = 32.0;
    private static final double kMaxDistanceForMinSpeedMeters = 3.0;
    private static final double kMinDistanceForMaxSpeedMeters = 6.0;
    private static final double kLauncherSpeedSlope = (kMaxLauncherSpeedMps - kMinLauncherSpeedMps) / (kMinDistanceForMaxSpeedMeters - kMaxDistanceForMinSpeedMeters);
    private static final double kFlywheelDiameter = 0.1016;
    private static final double kFlywheelGearRatio = 26.0 / 22.0;
    private static final double kAprilTagHeightMeters = 1.469;
    private static final double kFrontLimelightHeightMeters = 0.452;
    private static final double kAprilTagLimelightHeightDifferenceMeters = kAprilTagHeightMeters - kFrontLimelightHeightMeters;
    private static final double kFrontLimelightOffsetAngleDegrees = 24.89;
    private static final double kDistanceOffsetFromSpeakerTagToSpeakerOpeningMeters = 0.2664;
    private static final double kLaunchAxisOffsetMeters = 0.065;
    private static final double kLaunchExitOffsetMeters = 0.239;
    private static final double kDistanceFromLimelightToBotCenterMeters = 0.177306;
    private static final double kDistanceFromBotCenterToPivotMeters = 0.299;
    private static final Rotation2d kLiftAngle = Rotation2d.fromDegrees(10); 

    public enum LauncherHeightTarget {
        Floor(0.0),
        Speaker(2.078);
        public double heightMeters;
        LauncherHeightTarget (double heightMeters) {
            this.heightMeters = heightMeters;
        }
    }

    public static LauncherTarget getLauncherTarget(LauncherHeightTarget heightTarget, double liftHeightMeters, double distanceToTargetMeters, Rotation2d currentTiltAngle) {
        double adjustedDistanceMeters = distanceToTargetMeters + kDistanceFromLimelightToBotCenterMeters + kDistanceFromBotCenterToPivotMeters - getLauncherDistanceToPivotMeters(currentTiltAngle) + getLiftDistanceOffsetMeters(liftHeightMeters);
        double initialVelocityMPS = interpolateInitialVelocity(adjustedDistanceMeters);
        double adjustedLiftHeightMeters = Math.min(liftHeightMeters, getMinLiftHeightMetersForDistanceMeters(adjustedDistanceMeters));
        double height = heightTarget.heightMeters - kLauncherPivotHeightMeters - getLiftHeightOffsetMeters(adjustedLiftHeightMeters) - getLauncherHeightAbovePivotMeters(currentTiltAngle);
        double vSquared = Math.pow(initialVelocityMPS, 2);
        double sqrtExpression = Math.sqrt(Math.pow(initialVelocityMPS, 4) - kGravity * (kGravity*Math.pow(adjustedDistanceMeters, 2) + 2 * -height * Math.pow(initialVelocityMPS, 2)));
        double gx =  (kGravity * adjustedDistanceMeters);
        double expression = Math.abs((vSquared - sqrtExpression) / gx);

        double launcherRPM = mpsToLauncherRPM(initialVelocityMPS);
        double theta = Math.atan(expression);
        return new LauncherTarget(adjustedDistanceMeters, 0, launcherRPM, 0, Math.toDegrees(theta), adjustedLiftHeightMeters);
    }

    public static double interpolateInitialVelocity(double distance) {
        double interpolatedSpeed = (kLauncherSpeedSlope * (distance - kMaxDistanceForMinSpeedMeters)) + kMinLauncherSpeedMps;
        return MathUtil.clamp(interpolatedSpeed, kMinLauncherSpeedMps, kMaxLauncherSpeedMps);
    }

    public static double mpsToLauncherRPM(double mps) {
        double metersPerMinute = mps * 60;
        double wheelRPM = metersPerMinute / (Math.PI * kFlywheelDiameter);
        double drivenRPM = wheelRPM / kFlywheelGearRatio;
        return drivenRPM;
    }

    public static double getLauncherHeightAbovePivotMeters(Rotation2d currentTiltAngle) {
        // double launcherHeightAbovePivotMeters = 0.160;
        double tiltAngleRadians = currentTiltAngle.getRadians();

        // =(COS(RADIANS(I30))*J9) + (sin(RADIANS(I30))*J10)
        double launcherHeightAbovePivotMeters = (Math.cos(tiltAngleRadians) * kLaunchAxisOffsetMeters) + (Math.sin(tiltAngleRadians) * kLaunchExitOffsetMeters);

        return launcherHeightAbovePivotMeters;
    }

    public static double getLauncherDistanceToPivotMeters(Rotation2d currentTiltAngle) {
        // double launcherHeightAbovePivotMeters = 0.160;
        double tiltAngleRadians = currentTiltAngle.getRadians();

        //-(sin(RADIANS(I30))*J9) + (COS(RADIANS(I30))*J10)
        double launcherHeightAbovePivotMeters = -(Math.sin(tiltAngleRadians) * kLaunchAxisOffsetMeters) + (Math.cos(tiltAngleRadians) * kLaunchExitOffsetMeters);

        return launcherHeightAbovePivotMeters;
    }

    public static double getLiftHeightOffsetMeters(double currentLiftHeight) {
        // COS(RADIANS(10))*B21
        return Math.cos(kLiftAngle.getRadians()) * currentLiftHeight;
    }

    public static double getLiftDistanceOffsetMeters(double currentLiftHeight) {
        // SIN(RADIANS(10))*B21
        return Math.sin(kLiftAngle.getRadians()) * currentLiftHeight;
    }

    public static double speakerAprilTagTyToDistanceMeters(double ty) {
        double distanceMeters = kAprilTagLimelightHeightDifferenceMeters / Math.tan(Math.toRadians(kFrontLimelightOffsetAngleDegrees + ty));
        return distanceMeters - kDistanceOffsetFromSpeakerTagToSpeakerOpeningMeters;
    }

    public static double getMinLiftHeightMetersForDistanceMeters(double distanceMeters) {
        //-0.191 + 0.117x + -0.0141x^2 + 6.06E-04x^3
        if (distanceMeters < 2.167) {
            return LiftConstants.MinHeightMeters;
        }
        return -0.191 + (0.117 * distanceMeters) + (-0.0141 * Math.pow(distanceMeters, 2)) + (0.000606 * Math.pow(distanceMeters, 3));
    }
}
