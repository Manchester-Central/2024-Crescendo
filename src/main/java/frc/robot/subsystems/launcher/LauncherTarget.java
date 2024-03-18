package frc.robot.subsystems.launcher;

import java.text.DecimalFormat;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Add your docs here.
 */
public class LauncherTarget {
    protected final DecimalFormat m_formatter = new DecimalFormat("#.###");

    protected final double m_launcherSpeedRPM;
    protected final double m_speedOffsetRPM;
    protected final double m_tiltAngleDegrees;
    protected final double m_heightMeters;

    public LauncherTarget(double speed, double speedOffset, double launcherAngle, double heightMeters) {
        m_launcherSpeedRPM = speed;
        m_speedOffsetRPM = speedOffset;
        m_tiltAngleDegrees = launcherAngle;
        m_heightMeters = heightMeters;
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
        return String.join(", ", toDashboardValues());
    }

    public String[] toDashboardValues () {
        String[] values = {
            "RPM: " + m_formatter.format(m_launcherSpeedRPM),
            "RPM offset: " + m_formatter.format(m_speedOffsetRPM),
            "Tilt: " + m_formatter.format(m_tiltAngleDegrees),
            "Lift Height:" + m_formatter.format(m_heightMeters)
        };
        return values;
    }
}