package frc.robot.subsystems;

import java.util.Comparator;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class TableData {

    private double m_distanceMeters;
    private double m_launcherSpeedRPM;
    private double m_tiltAngleDegrees;
    private double m_heightMeters;

    public TableData(double distance, double speed, double launcherAngle, double heightMeters) {

        m_distanceMeters = distance;
        m_launcherSpeedRPM = speed;
        m_tiltAngleDegrees = launcherAngle;
        m_heightMeters = heightMeters;


    }

    public static TableData FromCSV(String[] args) throws Exception {
        var distanceMeters = getValueFromStringArray(args, 0, true, "distanceMeters");
        var launcherSpeedRPM = getValueFromStringArray(args, 1, true, "launcherSpeedRPM");
        var launcherAngle = getValueFromStringArray(args, 2, true, "tiltAngleDegrees");
        var heightMeters = getValueFromStringArray(args, 3, true, "heightMeters");

        return new TableData(
            Double.parseDouble(distanceMeters),
            Double.parseDouble(launcherSpeedRPM),
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

    public static Comparator<TableData> getComparator() {
        return new Comparator<TableData>() {
            @Override
            public int compare(TableData arg0, TableData arg1) {
                if (arg1.getDistanceMeters() == arg0.getDistanceMeters()){
                    return 0;
                }
                return arg1.getDistanceMeters() < arg0.getDistanceMeters() ? 1 : -1;
            }
        };
    }

    public double getDistanceMeters() {
        return m_distanceMeters;
    }

    public double getLauncherSpeedRPM() {
        return m_launcherSpeedRPM;
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