// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import java.util.function.Supplier;

import com.chaos131.logging.LogManager;
import com.chaos131.swerve.BaseSwerveDrive;
import com.chaos131.swerve.BaseSwerveModule;
import com.chaos131.swerve.SwerveConfigs;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants2024;
import frc.robot.util.Pose2dUtil;

public abstract class SwerveDrive extends BaseSwerveDrive {

    public SwerveDrive(BaseSwerveModule[] swerveModules, SwerveConfigs swerveConfigs,
            Supplier<Rotation2d> getRotation) {
        super(swerveModules, swerveConfigs, getRotation);
         AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::pathPlannerRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(4.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    swerveConfigs.maxRobotSpeed_mps(), // Max module speed, in m/s
                    0.3996, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
        );
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return m_kinematics.toChassisSpeeds(getModuleStates());
    }

    public void pathPlannerRobotRelative(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
        // SwerveDriveKinematics.desaturateWheelSpeeds(states, m_swerveConfigs.maxRobotSpeed_mps());
        for (var i = 0; i < states.length; i++) {
            m_swerveModules.get(i).setTarget(states[i]);
        }
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
