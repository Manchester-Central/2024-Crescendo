package frc.robot.subsystems;

import java.util.Comparator;

import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class TableData {

    private double m_distance;
    private double m_speed;
    private double m_launcherAngle;
    private double m_launcherTolerance;
    private double m_feederSpeed;

    public TableData(double distance, double speed, double launcherAngle, double launcherTolerance, double feederSpeed) {

        m_distance = distance;
        m_speed = speed;
        m_launcherAngle = launcherAngle;
        m_launcherTolerance = launcherTolerance;
        m_feederSpeed = feederSpeed;

    }

    public static TableData FromCSV(String[] args) throws Exception {
        var distance = getValueFromStringArray(args, 0, true, "distance");
        var speed = getValueFromStringArray(args, 1, true, "speed");
        var launcherAngle = getValueFromStringArray(args, 2, false, "launcherAngle");
        var launcherTolerance = getValueFromStringArray(args, 3, false, "launcherTolerance");
        var feederSpeed = getValueFromStringArray(args, 4, false, "feederSpeed");

        return new TableData(
            Double.parseDouble(distance),
            Double.parseDouble(speed),
            isValueSet(launcherAngle) ? Double.parseDouble(launcherAngle) : Constants.LauncherConstants.DefaultLauncherAngle,
            isValueSet(launcherTolerance) ? Double.parseDouble(launcherTolerance) : Constants.LauncherConstants.LauncherToleranceRPM,
            isValueSet(feederSpeed) ? Double.parseDouble(feederSpeed) : Constants.LauncherConstants.DefaultLauncherSpeed
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
                if (arg1.getDistance() == arg0.getDistance()){
                    return 0;
                }
                return arg1.getDistance() < arg0.getDistance() ? 1 : -1;
            }
        };
    }

    public double getDistance() {
        return m_distance;
    }

    public double getSpeed() {
        return m_speed;
    }
    
    public double getLauncherAngle() {
        return m_launcherAngle;
    }
    
    public double getLauncherTolerance() {
        return m_launcherTolerance;
    }
    
    public double getFeederSpeed() {
        return m_feederSpeed;
    }

    @Override
    public String toString() {
        return "distance=" + m_distance + ", speed = " + m_speed;
    }
}