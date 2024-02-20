// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.chaos131.poses.MirroredDrivePose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * A series of poses on the field for the robot to be to interact with the field mechanics.
 * 
 * <p>All poses use meters as the X/Y coordinates, with no vertical value.
 * 
 * <p>Most of the locations are April Tag locations, and not the location for the robot. 
 */
public class FieldPose2024 extends MirroredDrivePose {

    public static final double FieldWidthMeters = 16.512;
    public static final Alliance DefaultAlliance = Alliance.Blue;

    // The April Tag point at the Amp, accounting for robot orientation but not robot size as it tries to interact with the Amp
    public static final FieldPose2024 Amp           = new FieldPose2024("Amp", new Pose2d(1.842, 8.204, Rotation2d.fromDegrees(270)));
    // The BLUE teams's center chute's exit, and orientation the robot should be facing to pick up a note as it comes in (opposite side of the field)
    public static final FieldPose2024 Source        = new FieldPose2024("Source", new Pose2d(15.632, 0.5648, Rotation2d.fromDegrees(120)));
    // The rotation is the general angle the robot should be facing to see the speaker
    public static final FieldPose2024 Speaker       = new FieldPose2024("Speaker", new Pose2d(-0.0381, 5.547868, Rotation2d.fromDegrees(180)));
    
    // ID 15 and 12, this is the side of the stage that faces the Amp (with the blue robot rotation of course)
    public static final FieldPose2024 StageAmp      = new FieldPose2024("StageAmp", new Pose2d(4.641, 4.498, Rotation2d.fromDegrees(120)));
    // ID 14 and 13, this is the far side of the stage that's difficult to see, (with the blue robot rotation of course)
    public static final FieldPose2024 StageFar      = new FieldPose2024("StageFar", new Pose2d(5.321, 4.105, Rotation2d.fromDegrees(0)));
    // ID 16 and 11, this is the side of the stage that faces the source (with the blue robot rotation of course)
    public static final FieldPose2024 StageSource   = new FieldPose2024("StageSource", new Pose2d(4.641, 3.713, Rotation2d.fromDegrees(240)));

     // The April Tag point at the Amp, accounting for robot orientation but not robot size as it tries to interact with the Amp
    public static final FieldPose2024 TestStart     = new FieldPose2024("TestStart", new Pose2d(2, 3, Rotation2d.fromDegrees(0)));

    public FieldPose2024(String name, Pose2d bluePose) {
        super(FieldWidthMeters, DefaultAlliance, name, bluePose);
    }

    public FieldPose2024(Pose2d bluePose) {
        super(FieldWidthMeters, DefaultAlliance, null, bluePose);
    }

    public double distanceTo(Pose2d location) {
        return location.getTranslation().getDistance(getCurrentAlliancePose().getTranslation());
    }
}

