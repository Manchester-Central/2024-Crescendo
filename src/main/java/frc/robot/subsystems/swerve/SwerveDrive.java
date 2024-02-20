// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import java.util.function.Supplier;

import com.chaos131.swerve.BaseSwerveDrive;
import com.chaos131.swerve.BaseSwerveModule;
import com.chaos131.swerve.SwerveConfigs;

import edu.wpi.first.math.MathUtil;
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

    /**
     * Updates the speed modifiers [0.0, 1.0] of the robot for translation and rotation
     * @param translationSpeedModifier the value to multiply the swerve drive's translation output by
     * @param rotationSpeedModifier the value to multiply the swerve drive's rotation output by
     */
    public void updateSpeedModifier(double translationSpeedModifier, double rotationSpeedModifier) {
        BaseSwerveDrive.TranslationSpeedModifier = MathUtil.clamp(translationSpeedModifier, 0.0, 1.0);
        BaseSwerveDrive.RotationSpeedModifier = MathUtil.clamp(rotationSpeedModifier, 0.0, 1.0);
    }

    /**
     * Updates the speed modifier [0.0, 1.0] of the robot for translation and rotation
     * @param speedModifier the value to multiply the swerve drive's output by
     */
    public void updateSpeedModifier(double speedModifier) {
        var speedModifierClamped = MathUtil.clamp(speedModifier, 0.0, 1.0);
        BaseSwerveDrive.TranslationSpeedModifier = speedModifierClamped;
        BaseSwerveDrive.RotationSpeedModifier = speedModifierClamped;
    }
}
