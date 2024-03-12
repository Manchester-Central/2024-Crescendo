package frc.robot.subsystems;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Arrays;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

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
        assertEquals(29.8474, LauncherModel.getLauncherTarget(LauncherHeightTarget.Speaker, 0.09454154429, 3.327).getTiltAngle().getDegrees(), DELTA);
    }
}
