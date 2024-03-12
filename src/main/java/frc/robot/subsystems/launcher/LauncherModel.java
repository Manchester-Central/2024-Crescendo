// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

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

    public enum LauncherHeightTarget {
        Floor(0.0),
        Speaker(2.078);
        public double heightMeters;
        LauncherHeightTarget (double heightMeters) {
            this.heightMeters = heightMeters;
        }
    }

    public static LauncherTarget getLauncherTarget(LauncherHeightTarget heightTarget, double liftHeightMeters, double distanceToSpeakerMeters) {
        double initialVelocityMPS = interpolateInitialVelocity(distanceToSpeakerMeters);
        double height = heightTarget.heightMeters - kLauncherPivotHeightMeters - liftHeightMeters - getLauncherHeightAbovePivotMeters();
        double vSquared = Math.pow(initialVelocityMPS, 2);
        double sqrtExpression = Math.sqrt(Math.pow(initialVelocityMPS, 4) - kGravity * (kGravity*Math.pow(distanceToSpeakerMeters, 2) + 2 * -height * Math.pow(initialVelocityMPS, 2)));
        double gx =  (kGravity * distanceToSpeakerMeters);
        double expression = Math.abs((vSquared - sqrtExpression) / gx);

        double launcherRPM = mpsToLauncherRPM(initialVelocityMPS);
        double theta = Math.atan(expression);
        return new LauncherTarget(distanceToSpeakerMeters, 0, launcherRPM, 0, Math.toDegrees(theta), liftHeightMeters);
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

    public static double getLauncherHeightAbovePivotMeters() {
        double launcherHeightAbovePivotMeters = 0.160;
        return launcherHeightAbovePivotMeters;
    }

    public static double speakerAprilTagTyToDistanceMeters(double ty) {
        double distanceMeters = kAprilTagLimelightHeightDifferenceMeters / Math.tan(Math.toRadians(kFrontLimelightOffsetAngleDegrees + ty));
        return distanceMeters;
    }
}
