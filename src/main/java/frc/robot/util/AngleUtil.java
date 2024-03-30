// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.CameraDirection;

public class AngleUtil {

    public static Rotation2d GetEstimatedAngleToGoal(Vision camera, Pose2d currentPose, Rotation2d currentAngle) {
        var frontcam = camera.getCamera(CameraDirection.Front);
        if (frontcam.hasTarget()) {
            var aimXAngle = -frontcam.getTargetAzimuth(true);
            var aimCurrentAngle = currentAngle;
            var targetAngle = Rotation2d.fromDegrees(aimXAngle).plus(aimCurrentAngle).getDegrees();
            targetAngle = AngleUtil.clampAngle(targetAngle);
            return Rotation2d.fromDegrees(targetAngle);
        }
        return currentAngle;
    }

    public static double clampAngle(double angle) {
        return Math.IEEEremainder(angle, 360);
    }

}