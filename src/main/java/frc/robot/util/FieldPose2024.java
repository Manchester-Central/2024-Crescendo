// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.chaos131.poses.MirroredDrivePose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldPose2024 extends MirroredDrivePose {

    public static final double FieldWidthMeters = 16.512;
    public static final Alliance DefaultAlliance = Alliance.Blue;

    public static final FieldPose2024 Speaker = new FieldPose2024("Speaker", new Pose2d(-0.0381, 5.547868, Rotation2d.fromDegrees(0)));

    public FieldPose2024(String name, Pose2d bluePose) {
        super(FieldWidthMeters, DefaultAlliance, name, bluePose);
    }

    public FieldPose2024(Pose2d bluePose) {
        super(FieldWidthMeters, DefaultAlliance, null, bluePose);
    }

}