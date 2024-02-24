package frc.robot.commands.auto;

import com.chaos131.auto.ParsedCommand;
import com.chaos131.swerve.BaseSwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants;
import frc.robot.commands.DriveToLocation;
import frc.robot.commands.RunIntake;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Lift;
import frc.robot.util.FieldPose2024;

public class AutoUtil {
    public static double ParseDouble(String stringToParse, double defaultValue) {
        if(stringToParse == null) {
            return defaultValue;
        }

        return Double.parseDouble(stringToParse);
    }
    
    // /**
    //  * Gets the drive translation tolerance from the auto script step
    //  * @param parsedCommand the auto script step
    //  */
    // public static double getTranslationTolerance(ParsedCommand parsedCommand) {
    //     return AutoUtil.ParseDouble(parsedCommand.getArgument("translationTolerance"), SwerveConstants.DriveToTargetTolerance);
    // }
    
    // /**
    //  * Gets the mac percent speed from the auto script step
    //  * @param parsedCommand the auto script step
    //  */
    // public static double getMaxPercentSpeed(ParsedCommand parsedCommand) {
    //     return AutoUtil.ParseDouble(parsedCommand.getArgument("maxPercentSpeed"), SwerveConstants.MaxTranslationPIDSpeedPercent);
    // }

    /**
     * Gets the drive pose from the auto script step (if pose doesn't exist, it returns null)
     * @param parsedCommand the auto script step
     */
    public static Pose2d getDrivePose(ParsedCommand parsedCommand) {
        var poseName = parsedCommand.getArgument("drivePose");
        if(poseName != null && FieldPose2024.DrivePoses.containsKey(poseName)){
            return FieldPose2024.DrivePoses.get(poseName).getCurrentAlliancePose();
        }else if(poseName != null){
            return null; //If the drive pose we don't know about = do nothing
        }


        double x_meters = AutoUtil.ParseDouble(parsedCommand.getArgument("x"), 0.0);
        double y_meters = AutoUtil.ParseDouble(parsedCommand.getArgument("y"), 0.0);
        double angle_degrees = AutoUtil.ParseDouble(parsedCommand.getArgument("angle"), 0.0);
        return new Pose2d(x_meters, y_meters, Rotation2d.fromDegrees(angle_degrees));
    }

    public static Command driveAndIntake(ParsedCommand parsedCommand, BaseSwerveDrive swerveDrive, Intake intake, Lift lift, Launcher launcher, Feeder feeder){
        return DriveToLocation.createAutoCommand(parsedCommand, swerveDrive)
        .alongWith(RunIntake.createAutoCommand(parsedCommand, intake, lift, launcher, feeder));
    }

    public static Command driveAndIntakeSimple(ParsedCommand parsedCommand, BaseSwerveDrive swerveDrive, Intake intake, Lift lift, Launcher launcher, Feeder feeder){
        return DriveToLocation.createAutoCommand(parsedCommand, swerveDrive)
        .alongWith(new StartEndCommand(() -> intake.setIntakePower(0.7), () -> intake.setIntakePower(0), intake));
    }
}