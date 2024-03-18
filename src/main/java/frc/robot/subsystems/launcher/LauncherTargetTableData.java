package frc.robot.subsystems.launcher;

import java.util.Comparator;

public class LauncherTargetTableData extends LauncherTarget {
    protected final double m_distanceMeters;
    protected final double m_ty;

    public LauncherTargetTableData(double distance, double ty, double speed, double speedOffset,double launcherAngle, double heightMeters) {
        super(speed, speedOffset, launcherAngle, heightMeters);
        m_distanceMeters = distance;
        m_ty = ty;
    }

    public static LauncherTargetTableData FromCSV(String[] args) throws Exception {
        var distanceMeters = getValueFromStringArray(args, 0, true, "distanceMeters");
        var ty = getValueFromStringArray(args, 1, true, "ty");
        var launcherSpeedRPM = getValueFromStringArray(args, 2, true, "launcherSpeedRPM");
        var speedOffsetRPM = getValueFromStringArray(args, 3, true, "speedOffsetRPM");
        var launcherAngle = getValueFromStringArray(args, 4, true, "tiltAngleDegrees");
        var heightMeters = getValueFromStringArray(args, 5, true, "heightMeters");

        return new LauncherTargetTableData(
            Double.parseDouble(distanceMeters),
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

    public static Comparator<LauncherTargetTableData> getComparatorDistanceM() {
        return new Comparator<LauncherTargetTableData>() {
            @Override
            public int compare(LauncherTargetTableData arg0, LauncherTargetTableData arg1) {
                if (arg1.getDistanceMeters() == arg0.getDistanceMeters()){
                    return 0;
                }
                return arg1.getDistanceMeters() < arg0.getDistanceMeters() ? 1 : -1;
            }
        };
    }

    public static Comparator<LauncherTargetTableData> getComparatorTY() {
        return new Comparator<LauncherTargetTableData>() {
            @Override
            public int compare(LauncherTargetTableData arg0, LauncherTargetTableData arg1) {
                if (arg1.getTY() == arg0.getTY()){
                    return 0;
                }
                return arg1.getTY() < arg0.getTY() ? 1 : -1;
            }
        };
    }

    public double getDistanceMeters() {
        return m_distanceMeters;
    }

    public double getTY() {
        return m_ty;
    }


    @Override
    public String toString() {
        return String.join(", ", toDashboardValues());
    }

    @Override
    public String[] toDashboardValues() {
        String[] values = {
            "Distance: " + m_formatter.format(m_distanceMeters),
            "TY: " + m_formatter.format(m_ty),
            "RPM: " + m_formatter.format(m_launcherSpeedRPM),
            "RPM offset: " + m_formatter.format(m_speedOffsetRPM),
            "Tilt: " + m_formatter.format(m_tiltAngleDegrees),
            "Lift Height:" + m_formatter.format(m_heightMeters)
        };
        return values;
    }
}