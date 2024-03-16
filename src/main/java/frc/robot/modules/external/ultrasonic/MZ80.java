// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.modules.external.ultrasonic;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MZ80 extends SubsystemBase {
    private static MZ80 instance = null;

    private final DigitalInput sensor;

    public MZ80() {
        sensor = new DigitalInput(7);
    }

    public static MZ80 getInstance() {
        if (instance == null) {
            instance = new MZ80();
        }
        return instance;
    }

    public boolean isSenorDistanceReached() {
        return !sensor.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("dz80", !sensor.get());
    }
}
