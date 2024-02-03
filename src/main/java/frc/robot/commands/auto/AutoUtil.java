package frc.robot.commands.auto;

import com.chaos131.auto.ParsedCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

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
        double x_meters = AutoUtil.ParseDouble(parsedCommand.getArgument("x"), 0.0);
        double y_meters = AutoUtil.ParseDouble(parsedCommand.getArgument("y"), 0.0);
        double angle_degrees = AutoUtil.ParseDouble(parsedCommand.getArgument("angle"), 0.0);
        return new Pose2d(x_meters, y_meters, Rotation2d.fromDegrees(angle_degrees));
    }

}