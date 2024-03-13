package frc.robot.subsystems;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Arrays;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.launcher.FlywheelTable;
import frc.robot.subsystems.launcher.LauncherModel;
import frc.robot.subsystems.launcher.LauncherModel.LauncherHeightTarget;

public class LauncherModelTests {
    double DELTA = 0.001;

    @Test
    public void testInterpolateInitialVelocity() {
        assertEquals(18.8, LauncherModel.interpolateInitialVelocity(3), DELTA);
        assertEquals(32, LauncherModel.interpolateInitialVelocity(6), DELTA);

        assertEquals(18.8, LauncherModel.interpolateInitialVelocity(0), DELTA);
        assertEquals(32, LauncherModel.interpolateInitialVelocity(20), DELTA);

        assertEquals(25.4, LauncherModel.interpolateInitialVelocity(4.5), DELTA);
    }

    @Test
    public void testGetLauncherTarget() {
        assertEquals(26.973, LauncherModel.getLauncherTarget(LauncherHeightTarget.Speaker, 0.09454154429, 3.327, LauncherConstants.MinAngle).getTiltAngle().getDegrees(), DELTA);
    }

    @Test
    public void testRPMConversion() {
        assertEquals(3000, LauncherModel.mpsToLauncherRPM(18.86), DELTA+5);
    }

    @Test
    public void pleaseLetJoshGoHome() {
        assertEquals(3.74, LauncherModel.speakerAprilTagTyToDistanceMeters(-10.63), 0.01);
    }

    @Test
    public void testLauncherLiftHeight() {
        assertEquals(0.002493615927, LauncherModel.getMinLiftHeightMetersForDistanceMeters(2.167), DELTA);
        assertEquals(0.137675494, LauncherModel.getMinLiftHeightMetersForDistanceMeters(6.272), DELTA);
    }
}
