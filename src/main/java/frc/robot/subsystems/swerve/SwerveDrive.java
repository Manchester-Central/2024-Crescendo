// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import java.util.function.Supplier;

import com.chaos131.swerve.BaseSwerveDrive;
import com.chaos131.swerve.BaseSwerveModule;
import com.chaos131.swerve.SwerveConfigs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Pose2dUtil;

public abstract class SwerveDrive extends BaseSwerveDrive {

    public SwerveDrive(BaseSwerveModule[] swerveModules, SwerveConfigs swerveConfigs,
            Supplier<Rotation2d> getRotation) {
        super(swerveModules, swerveConfigs, getRotation);
    }
    /**
     * Translates the robot's current pose with forward moving in the direction of the current angle and left moving orthogonal to the current angle
     * @param robotForwardMeters the meters to move forward (or backwards if negative) in relation to the current pose and direction
     * @param robotLeftMeters the meters to move left (or right if negative) in relation to the current pose and direction
     */
    public Pose2d getTranslatedPose(double robotForwardMeters, double robotLeftMeters) {
        return Pose2dUtil.getTranslatedPose(getPose(), robotForwardMeters, robotLeftMeters);
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Odometry Angle Degrees", getOdometryRotation().getDegrees());
    }
}
