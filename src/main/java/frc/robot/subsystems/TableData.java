package frc.robot.subsystems;

import java.util.Comparator;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class TableData {

    private double m_distanceMeters;
    private double m_ty;
    private double m_launcherSpeedRPM;
    private double m_speedOffsetRPM;
    private double m_tiltAngleDegrees;
    private double m_heightMeters;

    public TableData(double ty, double speed, double speedOffset,double launcherAngle, double heightMeters) {

        // m_distanceMeters = distance;
        m_ty = ty;
        m_launcherSpeedRPM = speed;
        m_speedOffsetRPM = speedOffset;
        m_tiltAngleDegrees = launcherAngle;
        m_heightMeters = heightMeters;

    }

    public static TableData FromCSV(String[] args) throws Exception {
        // var distanceMeters = getValueFromStringArray(args, 0, true, "distanceMeters");
        var ty = getValueFromStringArray(args, 1, true, "ty");
        var launcherSpeedRPM = getValueFromStringArray(args, 2, true, "launcherSpeedRPM");
        var speedOffsetRPM = getValueFromStringArray(args, 3, true, "speedOffsetRPM");
        var launcherAngle = getValueFromStringArray(args, 4, true, "tiltAngleDegrees");
        var heightMeters = getValueFromStringArray(args, 5, true, "heightMeters");

        return new TableData(
            // Double.parseDouble(distanceMeters),
            Double.parseDouble(ty),
            Double.parseDouble(launcherSpeedRPM),
            Double.parseDouble(speedOffsetRPM),
            Double.parseDouble(launcherAngle), 
            Double.parseDouble(heightMeters)
        );
    }

    private static String getValueFromStringArray(String[] args, int index, boolean isRequired, String columnName) throws Exception {
        String value = null;
        if(args.length >= index + 1) {
            value = args[index];
        }

        if(!isValueSet(value) && isRequired) {
            throw new Exception("Missing required value from CSV: " + columnName);
        }
        return value;
    }

    private static boolean isValueSet(String value) {
        return value != null && !value.isBlank();
    }

    // public static Comparator<TableData> getComparatorDistanceM() {
    //     return new Comparator<TableData>() {
    //         @Override
    //         public int compare(TableData arg0, TableData arg1) {
    //             if (arg1.getDistanceMeters() == arg0.getDistanceMeters()){
    //                 return 0;
    //             }
    //             return arg1.getDistanceMeters() < arg0.getDistanceMeters() ? 1 : -1;
    //         }
    //     };
    // }

    public static Comparator<TableData> getComparatorTY() {
        return new Comparator<TableData>() {
            @Override
            public int compare(TableData arg0, TableData arg1) {
                if (arg1.getTY() == arg0.getTY()){
                    return 0;
                }
                return arg1.getTY() < arg0.getTY() ? 1 : -1;
            }
        };
    }

    // public double getDistanceMeters() {
    //     return m_distanceMeters;
    // }

    public double getTY() {
        return m_ty;
    }

    public double getLauncherSpeedRPM() {
        return m_launcherSpeedRPM;
    }

    public double getSpeedOffsetRPM(){
        return m_speedOffsetRPM;
    }
    
    public Rotation2d getTiltAngle() {
        return Rotation2d.fromDegrees(m_tiltAngleDegrees);
    }
       
    public double getHeightMeters() {
        return m_heightMeters;
    }

    @Override
    public String toString() {
        return "distance = " + m_distanceMeters + ", speed = " + m_launcherSpeedRPM + ", tilt = " + m_tiltAngleDegrees + ", height = " + m_heightMeters;
    }
}