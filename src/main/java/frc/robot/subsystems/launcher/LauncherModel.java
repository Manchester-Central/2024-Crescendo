// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import java.util.Optional;

import com.chaos131.util.DashboardNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DebugConstants;
import frc.robot.Constants.LauncherConstants;
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
    private static final double kFrontLimelightOffsetAngleDegrees = 24.84; // 24.75
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
    private static final double kMaxLauncherSpeedMps = 30.0; // original 32.0
    private static final double kMaxDistanceForMinSpeedMeters = 3.0;
    private static final double kMinDistanceForMaxSpeedMeters = 6.0;
    private static final double kLauncherSpeedSlope = (kMaxLauncherSpeedMps - kMinLauncherSpeedMps) / (kMinDistanceForMaxSpeedMeters - kMaxDistanceForMinSpeedMeters);

    // Flywheel to motor speeds constants
    private static final double kFlywheelDiameter = 0.1016;
    private static final double kFlywheelGearRatio = 26.0 / 22.0;
    private static final double kMaxLaunchDistanceMeters = 9.0;

    // Efficiency Lost Multiplier
    private static final double kEfficiencyLossMultiplierClose = 0.6;
    private static final double kEfficiencyLossMultiplierFar = 0.6; // TO-DO: tune me!
    private static final double kCloseDistance = 4; // TO-DO: tune me!
    private static final double kFarDistance = 8; // TO-DO: tune me!
    private static final DashboardNumber m_efficiencyLossMultiplierClose = new DashboardNumber("LauncherModel/Efficiency Loss Multiplier Close", kEfficiencyLossMultiplierClose, DebugConstants.LauncherModelDebugEnable, (newValue) -> {});
    private static final DashboardNumber m_efficiencyLossMultiplierFar = new DashboardNumber("LauncherModel/Efficiency Loss Multiplier Far", kEfficiencyLossMultiplierFar, DebugConstants.LauncherModelDebugEnable, (newValue) -> {});
    private static final DashboardNumber m_closeDistance = new DashboardNumber("LauncherModel/Close Distance", kCloseDistance, DebugConstants.LauncherModelDebugEnable, (newValue) -> {});
    private static final DashboardNumber m_farDistance = new DashboardNumber("LauncherModel/Far Distance", kFarDistance, DebugConstants.LauncherModelDebugEnable, (newValue) -> {});
    
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

    /**
     * Whether to get the higher or lower angle from the quadratic calculation
     */
    public enum TargetAngleMode {
        Higher, Lower
    }

    /**
     * Gets the launcher target for aiming at a height target (floor or speaker) for a given distance.
     * NOTE: This is working well for the speaker, but the velocity values are more sensitive for the floor, so it's recommended to use getLauncherTargetWithAngle for launching to the floor.
     * @param heightTarget Floor or Speaker, whichever you want to target.
     * @param currentLiftHeightMeters The current (or min) height for the lift
     * @param distanceToTargetFromBotCenterMeters the distance in meters from the bot center to the target
     * @param currentTiltAngle the current tilt of the launcher
     * @param targetAngleMode whether to prefer the possible higher or lower angle for the launcher target
     * @return An empty Optional if the resultant angles are not in the launcher range - otherwise, the resultant launcher target
     */
    public static Optional<LauncherTarget> getLauncherTarget(LauncherHeightTarget heightTarget, double currentLiftHeightMeters, double distanceToTargetFromBotCenterMeters, Rotation2d currentTiltAngle, TargetAngleMode targetAngleMode) {
        // Calculate the distance from the launch point to the target
        double adjustedDistanceMeters = distanceToTargetFromBotCenterMeters + kDistanceFromBotCenterToPivotMeters - getLauncherDistanceToPivotMeters(currentTiltAngle) + getLiftDistanceOffsetMeters(currentLiftHeightMeters);

        // Adjust the height to at least be above the min height calculated for the current distance
        //double adjustedLiftHeightMeters = Math.max(currentLiftHeightMeters, getMinLiftHeightMetersForDistanceMeters(adjustedDistanceMeters));
        double adjustedLiftHeightMeters = Math.max(currentLiftHeightMeters, getMinLiftHeightMetersForTiltAngle(currentTiltAngle));

        // Calculate the initial velocity target for the flywheels
        // These velocities do not work well with launching to the floor - using the getLauncherTargetWithAngle works better
        double initialVelocityMPS = interpolateInitialVelocityMps(adjustedDistanceMeters);

        // Calculate the height difference from the launch point to the target (positive for speaker, negative for floor)
        double launchHeightDifferenceMeters = heightTarget.heightMeters - kLauncherPivotHeightMeters - getLiftHeightOffsetMeters(adjustedLiftHeightMeters) - getLauncherHeightAbovePivotMeters(currentTiltAngle);

        // Run the calculation with a slightly lower velicty to account for energy transfer loss
        double adjustedVelocityMps = initialVelocityMPS * interpolateDouble(adjustedDistanceMeters, kCloseDistance, kFarDistance, m_efficiencyLossMultiplierClose.get(),
                                                                            m_efficiencyLossMultiplierFar.get());

        // Calculate the desired launcher angle from the calculated values
        // Using the formula Dan gave us here: https://docs.google.com/spreadsheets/d/1uqDhTsbwMxrkkyMMzWEmLbcNDx1oy95dRZYe3OpYG2s/edit?usp=sharing
        double vSquared = Math.pow((adjustedVelocityMps), 2);
        double sqrtExpression = Math.sqrt(Math.pow(adjustedVelocityMps, 4) - kGravity * (kGravity*Math.pow(adjustedDistanceMeters, 2) + 2 * -launchHeightDifferenceMeters * Math.pow(adjustedVelocityMps, 2)));
        double gx =  (kGravity * adjustedDistanceMeters);
        double expressionA = Math.abs((vSquared - sqrtExpression) / gx);
        double expressionB = Math.abs((vSquared + sqrtExpression) / gx);
        double thetaRadsA = Math.atan(expressionA);
        double thetaRadsB = Math.atan(expressionB);
        double thetaDegreesA = Math.toDegrees(thetaRadsA);
        double thetaDegreesB = Math.toDegrees(thetaRadsB);

        // Convert flywheel speed (mps) to the RPM we need for the launcher
        double launcherRPM = mpsToLauncherRPM(initialVelocityMPS);

        double higherThetaDegrees = thetaDegreesA > thetaDegreesB ? thetaDegreesA : thetaDegreesB;
        double lowerThetaDegrees = thetaDegreesA < thetaDegreesB ? thetaDegreesA : thetaDegreesB;
        boolean isHigherAngleValid = higherThetaDegrees < LauncherConstants.MaxAngle.getDegrees() && higherThetaDegrees > LauncherConstants.MinAngle.getDegrees();
        boolean isLowerAngleValid = lowerThetaDegrees < LauncherConstants.MaxAngle.getDegrees() && lowerThetaDegrees > LauncherConstants.MinAngle.getDegrees(); 

        var higherLauncherTarget = new LauncherTarget(launcherRPM, 0, higherThetaDegrees, adjustedLiftHeightMeters);
        var lowerLauncherTarget = new LauncherTarget(launcherRPM, 0, lowerThetaDegrees, adjustedLiftHeightMeters);

        publishTargetToDashboard(Optional.of(higherLauncherTarget), "Possible Target Higher");
        publishTargetToDashboard(Optional.of(lowerLauncherTarget), "Possible Target Lower");

        var result = Optional.ofNullable((LauncherTarget) null);
        if(!isHigherAngleValid && !isLowerAngleValid){
            result = Optional.empty();
        } else if(!isHigherAngleValid){
            result = Optional.of(lowerLauncherTarget);
        } else if(!isLowerAngleValid){
            result = Optional.of(higherLauncherTarget);
        } else {
            result = targetAngleMode == TargetAngleMode.Higher ? Optional.of(higherLauncherTarget) : Optional.of(lowerLauncherTarget);
        }
        publishTargetToDashboard(result, "Launch Target");
        return result;
    }

    /**
     * Gets the launcher target for a given distance and set tilt angle (given an angle, the flywheel velocity is the changing variable)
     * Note this has currently only been tested for launching to the floor.
     * @param heightTarget whether aiming for the floor or speaker
     * @param liftHeightMeters the current height of the lift
     * @param distanceToTargetFromBotCenterMeters the distance in meters from the bot center to the target
     * @param targetAngle the target tilt angle for launching
     * @return An empty Optional if the resultant velocity is greater than the max velocity - otherwise, the resultant launcher target
     */
    public static Optional<LauncherTarget> getLauncherTargetWithAngle(LauncherHeightTarget heightTarget, double liftHeightMeters, double distanceToTargetFromBotCenterMeters, Rotation2d targetAngle) {
        // Calculate the distance from the launch point to the target
        double adjustedDistanceMeters = distanceToTargetFromBotCenterMeters + kDistanceFromBotCenterToPivotMeters - getLauncherDistanceToPivotMeters(targetAngle) + getLiftDistanceOffsetMeters(liftHeightMeters);
        adjustedDistanceMeters = Math.min(adjustedDistanceMeters, kMaxLaunchDistanceMeters);

        // Adjust the height to at least be above the min height calculated for the current distance
        double adjustedLiftHeightMeters = Math.max(liftHeightMeters, getMinLiftHeightMetersForTiltAngle(targetAngle));

        // Calculate the height difference from the launch point to the target (positive for speaker, negative for floor)
        double launchHeightDifferenceMeters = heightTarget.heightMeters - kLauncherPivotHeightMeters - getLiftHeightOffsetMeters(adjustedLiftHeightMeters) - getLauncherHeightAbovePivotMeters(targetAngle);

        // Calculate the desired velocity from the calculated values
        // Using the formula Dan gave us here: https://en.wikipedia.org/wiki/Projectile_motion#Displacement
        var numerator = Math.pow(adjustedDistanceMeters, 2) * -kGravity;
        var denominator = (adjustedDistanceMeters * Math.sin(2 * targetAngle.getRadians())) - (2 * launchHeightDifferenceMeters * Math.pow(Math.cos(targetAngle.getRadians()), 2));
        var initialVelocityMPS = Math.sqrt(numerator / denominator) * 2; // TODO: why did we need to multiply by two??

        // Convert flywheel speed (mps) to the RPM we need for the launcher
        double launcherRPM = mpsToLauncherRPM(initialVelocityMPS);

        var launcherTarget = new LauncherTarget(launcherRPM, 0, targetAngle.getDegrees(), adjustedLiftHeightMeters);
        var result = Optional.of(launcherTarget);

        // If the velocity is too high, we can't actually launch
        if (launcherRPM > LauncherConstants.MaxRPM) {
            result = Optional.empty();
        }

        publishTargetToDashboard(result, "Launch Target");

        return result;
    }

    /**
     * Publishes the targets to the dashboard, if LauncherModelDebugEnable is enabled
     * @param targets the targets to report
     * @param dashboardName the name to be appended to "LauncherModel/"
     */
    public static void publishTargetToDashboard(Optional<LauncherTarget> targets, String dashboardName) {
        var fullDashboardName = "LauncherModel/" + dashboardName;
        if (!DebugConstants.LauncherModelDebugEnable) {
            return;
        }

        if (targets.isEmpty()) {
            SmartDashboard.putStringArray(fullDashboardName, new String[] {"No target"});
            return;
        }

        SmartDashboard.putStringArray(fullDashboardName, targets.get().toDashboardValues());
    }

    /**
     * Calculate the flywheel speed (mps) for the given distance to the goal
     * @param distance the distance to the goal
     */
    public static double interpolateInitialVelocityMps(double distance) {
        double interpolatedSpeed = (kLauncherSpeedSlope * (distance - kMaxDistanceForMinSpeedMeters)) + kMinLauncherSpeedMps;
        return MathUtil.clamp(interpolatedSpeed, kMinLauncherSpeedMps, kMaxLauncherSpeedMps);
    }

    // x: distance
    // y0 = minAdjustedSpeed
    // y1 = maxAdjustedSpeed
    // m = slope between y0 and y1

    public static double interpolateDouble(double input, double x0, double x1, double y0, double y1) {
        double slope = (y1 - y0) / (x1 - x0);
        double interpolatedValue = y0 + slope * (input - x0);
        return MathUtil.clamp(interpolatedValue, y0, y1);
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
    public static double speakerOpeningToBotCenterDistanceMetersByTY(double ty) {
        double distanceMeters = speakerAprilTagToLimelightDistanceMetersByTY(ty);
        return distanceMeters + kDistanceFromLimelightToBotCenterMeters - kDistanceOffsetFromSpeakerTagToSpeakerOpeningMeters;
    }

    public static double speakerAprilTagToLimelightDistanceMetersByTY(double ty) {
        return kAprilTagLimelightHeightDifferenceMeters / Math.tan(Math.toRadians(kFrontLimelightOffsetAngleDegrees + ty));
    }

    public static double speakerAprilTagToBotCenterDistanceMetersByTY(double ty) {
        return speakerAprilTagToLimelightDistanceMetersByTY(ty) + kDistanceFromLimelightToBotCenterMeters;
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

    /**
     * Gets the min lift height for a given tilt angle to the goal
     */
    public static double getMinLiftHeightMetersForTiltAngle(Rotation2d tiltAngle) {
        double tiltAngleDegrees = tiltAngle.getDegrees();
        //=0.214-0.00602*A2-0.0000222*POW(A2,2)
        double calculatedHeight = 0.214 + (-0.00602 * tiltAngleDegrees) + (-0.0000222 * Math.pow(tiltAngleDegrees, 2));
        return MathUtil.clamp(calculatedHeight, LiftConstants.MinHeightMeters, LiftConstants.MaxHeightMeters);
    }
}
