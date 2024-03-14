// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Arrays;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import frc.robot.subsystems.launcher.FlywheelTable;

/** Add your docs here. */
public class FlywheelTableTests {
    double DELTA = 0.001;
    FlywheelTable m_flywheelTable;

    @BeforeEach
    public void clean() {    
        String[] cvsLines = {
            "0,0,1000,100,10,1",
            "5,5,5000,150,20,0.5",
            "10,10,10000,200,30,0.1",
        };
        m_flywheelTable = new FlywheelTable(Arrays.asList(cvsLines));
    }

    @Test
    public void testOutOfBounds() {
        assertTrue(m_flywheelTable.getIdealTargetByTY(-1).isEmpty());
        assertTrue(m_flywheelTable.getIdealTargetByTY(10.1).isEmpty());
    }

    @Test
    public void testInBounds() {
        assertTrue(m_flywheelTable.getIdealTargetByTY(0.0).isPresent());
        assertTrue(m_flywheelTable.getIdealTargetByTY(0.1).isPresent());
        assertTrue(m_flywheelTable.getIdealTargetByTY(5).isPresent());
        assertTrue(m_flywheelTable.getIdealTargetByTY(9.1).isPresent());
        assertTrue(m_flywheelTable.getIdealTargetByTY(10).isPresent());
    }

    @Test
    public void testInterpolatedSpeed() {
        assertEquals(1000, m_flywheelTable.getIdealTargetByTY(0.0).get().getLauncherSpeedRPM(), DELTA);
        assertEquals(3000, m_flywheelTable.getIdealTargetByTY(2.5).get().getLauncherSpeedRPM(), DELTA);
        assertEquals(5000, m_flywheelTable.getIdealTargetByTY(5.0).get().getLauncherSpeedRPM(), DELTA);
    }

    @Test
    public void testInterpolatedTilt() {
        assertEquals(10, m_flywheelTable.getIdealTargetByTY(0.0).get().getTiltAngle().getDegrees(), DELTA);
        assertEquals(15, m_flywheelTable.getIdealTargetByTY(2.5).get().getTiltAngle().getDegrees(), DELTA);
        assertEquals(20, m_flywheelTable.getIdealTargetByTY(5.0).get().getTiltAngle().getDegrees(), DELTA);
    }

    @Test
    public void testDiscreteOffset() {
        assertEquals(100, m_flywheelTable.getIdealTargetByTY(0.0).get().getSpeedOffsetRPM(), DELTA);
        assertEquals(100, m_flywheelTable.getIdealTargetByTY(2.5).get().getSpeedOffsetRPM(), DELTA);
        assertEquals(150, m_flywheelTable.getIdealTargetByTY(5.0).get().getSpeedOffsetRPM(), DELTA);
    }

    @Test
    public void testDiscreteHeight() {
        assertEquals(1.0, m_flywheelTable.getIdealTargetByTY(0.0).get().getHeightMeters(), DELTA);
        assertEquals(1.0, m_flywheelTable.getIdealTargetByTY(2.5).get().getHeightMeters(), DELTA);
        assertEquals(0.5, m_flywheelTable.getIdealTargetByTY(5.0).get().getHeightMeters(), DELTA);
    }
}
