// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.QuestNav;
import frc.robot.Constants.QuestConstants;
import frc.robot.subsystems.swerve.SwerveDrive;

public class Quest extends SubsystemBase {
  private QuestNav m_questNav = new QuestNav();
  private SwerveDrive m_swerveDrive;

  public Quest(SwerveDrive swerveDrive) {
    m_swerveDrive = swerveDrive;
  }

  @Override
  public void periodic() {
    m_questNav.cleanupResponses();
    m_questNav.processHeartbeat();

    if (DriverStation.isEnabled()) {
      Pose2d questPose = m_questNav.getPose();
      Pose2d robotPose = questPose.transformBy(QuestConstants.QuestToRobot.inverse());
    } else {
      Pose2d robotPose = m_swerveDrive.getPose();
      Pose2d questPose = m_questNav.getPose();
      m_questNav.setPose(robotPose);
    }
    


    Matrix<N3, N1> QUESTNAV_STD_DEVS =
    VecBuilder.fill(
        0.02, // Trust down to 2cm in X direction
        0.02, // Trust down to 2cm in Y direction
        0.035 // Trust down to 2 degrees rotational
    );

    if (m_questNav.getConnected() && m_questNav.getTrackingStatus()) {
        // Get pose with the method outlined above
        Pose2d pose = m_questNav.getPose();
        // Get timestamp from the QuestNav instance
        double timestamp = m_questNav.getTimestamp();

        // You can put some sort of filtering here if you would like!

        // Add the measurement to our estimator
        m_swerveDrive.addVisionMeasurement(pose, timestamp, QUESTNAV_STD_DEVS);
    }
  }
}
