// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.chaos131.gamepads.Gamepad;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class RumbleManager {
    private Gamepad m_driver;
    private Gamepad m_operator;
    private Feeder m_feeder;
    private Intake m_intake;

    private double m_noteSeenTime = 0;
    private boolean m_noteRumbleDebounce = false;

    public RumbleManager(Gamepad driver, Gamepad operator, Feeder feeder, Intake intake) {
        m_driver = driver;
        m_operator = operator;
        m_feeder = feeder;
        m_intake = intake;
    }

    public void enableRumble() {
        handleDriverRumble();
        handleOperatorRumble();
    }

    // Rumble the driver controller inversely proportional to the battery voltage
    // (up to a certain point) (so rumber more the lower the reported voltage gets)
    private void handleDriverRumble() {
        if ((m_feeder.hasNoteAtSecondary() || m_intake.hasNote()) && DriverStation.isTeleop()) {
            m_driver.getHID().setRumble(RumbleType.kBothRumble, 1.0);
        } else {
            m_driver.getHID().setRumble(RumbleType.kBothRumble, 0);
        }
    }

    // Rumble the operator controller for 0.25 seconds after getting a note and then
    // stop until the next time
    private void handleOperatorRumble() {

        // If this is the first time seeing this note in the intake
        if ((m_feeder.hasNote()) && m_noteRumbleDebounce == false) {
            m_operator.getHID().setRumble(RumbleType.kBothRumble, 1.0);
            m_noteSeenTime = Timer.getFPGATimestamp();
            m_noteRumbleDebounce = true;
        }

        // If we have no note or it's been more than 250` milliseconds since we first
        // saw this note
        if (Timer.getFPGATimestamp() - m_noteSeenTime >= 0.25 && m_noteRumbleDebounce == true) {
            m_operator.getHID().setRumble(RumbleType.kBothRumble, 0);
        }

        if (!m_feeder.hasNote()) {
            m_operator.getHID().setRumble(RumbleType.kBothRumble, 0);
            m_noteRumbleDebounce = false;
        }
    }

    public void disableRumble() {
        m_driver.getHID().setRumble(RumbleType.kBothRumble, 0.0);
        m_operator.getHID().setRumble(RumbleType.kBothRumble, 0.0);
    }
}
