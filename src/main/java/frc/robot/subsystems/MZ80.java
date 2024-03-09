// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
//import edu.wpi.first.wpilibj.AnalogInput;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MZ80 extends SubsystemBase {

    private static MZ80 instance = null;

    public static MZ80 getInstance() {
        if (instance == null) {
            instance = new MZ80();
        }
        return instance;
    }

    /**
     * Creates a new dz80.
     */
    // private final AnalogInput sensorA;
    private final DigitalInput sensor;

    public MZ80() {
        sensor = new DigitalInput(9);
        // sensorA = new AnalogInput(0);
    }

    public boolean sensorget() {
        return !sensor.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("dz80", !sensor.get());
        //  SmartDashboard.putNumber("analog get", sensorA.getVoltage());
        // This method will be called once per scheduler run
    }
}
