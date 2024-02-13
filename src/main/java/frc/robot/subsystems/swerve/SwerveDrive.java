// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import java.util.function.Supplier;

import com.chaos131.swerve.BaseSwerveDrive;
import com.chaos131.swerve.BaseSwerveModule;
import com.chaos131.swerve.SwerveConfigs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class SwerveDrive extends BaseSwerveDrive {

    public SwerveDrive(BaseSwerveModule[] swerveModules, SwerveConfigs swerveConfigs,
            Supplier<Rotation2d> getRotation) {
        super(swerveModules, swerveConfigs, getRotation);
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Odometry Angle Degrees", getOdometryRotation().getDegrees());
    }
}
