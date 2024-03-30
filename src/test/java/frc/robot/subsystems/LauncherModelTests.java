package frc.robot.subsystems;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.LiftConstants;
import frc.robot.subsystems.launcher.LauncherModel;
import frc.robot.subsystems.launcher.LauncherModel.LauncherHeightTarget;
import frc.robot.subsystems.launcher.LauncherModel.TargetAngleMode;

public class LauncherModelTests {
    double DELTA = 0.001;

    @Test
    public void testInterpolateInitialVelocity() {
        assertEquals(18.8, LauncherModel.interpolateInitialVelocityMps(3), DELTA);
        assertEquals(30, LauncherModel.interpolateInitialVelocityMps(6), DELTA);

        assertEquals(18.8, LauncherModel.interpolateInitialVelocityMps(0), DELTA);
        assertEquals(30, LauncherModel.interpolateInitialVelocityMps(20), DELTA);

        assertEquals(24.4, LauncherModel.interpolateInitialVelocityMps(4.5), DELTA);
    }

    @Test
    public void testGetLauncherTarget() {
        assertEquals(27.732, LauncherModel.getLauncherTarget(LauncherHeightTarget.Speaker, 0.09454154429, 3.327, LauncherConstants.MinAngle, TargetAngleMode.Lower).get().getTiltAngle().getDegrees(), DELTA);
    }

    @Test
    public void testRPMConversion() {
        assertEquals(3000, LauncherModel.mpsToLauncherRPM(18.86), DELTA+5);
    }

    @Test
    public void testSpeakerTyToDistanceMeters() {
        assertEquals(3.927, LauncherModel.speakerAprilTagTyToBotCenterDistanceMeters(-10.63), 0.01);
    }

    @Test
    public void testLauncherLiftHeight() {
        assertEquals(LiftConstants.MinHeightMeters, LauncherModel.getMinLiftHeightMetersForDistanceMeters(1), DELTA);
        assertEquals(LiftConstants.MinHeightMeters, LauncherModel.getMinLiftHeightMetersForDistanceMeters(2.167), DELTA);
        assertEquals(0.137675494, LauncherModel.getMinLiftHeightMetersForDistanceMeters(6.272), DELTA);
        assertEquals(0.1593999813, LauncherModel.getMinLiftHeightMetersForDistanceMeters(8.772), DELTA);
    }
}
