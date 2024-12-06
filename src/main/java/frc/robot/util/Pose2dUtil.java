// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Pose2dUtil {
    private Pose2dUtil() {}

    /**
     * Translates the pose (with the given Rotation2d) with forward moving in the direction of the current angle and left moving orthogonal to the current angle
     * @param originalPose the current pose with x meters, y meters, and a Rotation2d
     * @param forwardMeters the meters to move forward (or backwards if negative) in relation to the current pose and direction
     * @param leftMeters the meters to move left (or right if negative) in relation to the current pose and direction
     */
    public static Pose2d getTranslatedPose(Pose2d originalPose, double forwardMeters, double leftMeters) {
        return originalPose.plus(new Transform2d(new Translation2d(forwardMeters, leftMeters), Rotation2d.fromDegrees(0)));
    }

    public static double getDistanceMeters(Pose2d startPose, Pose2d endPose) {
        return startPose.getTranslation().getDistance(endPose.getTranslation());
    }
}

