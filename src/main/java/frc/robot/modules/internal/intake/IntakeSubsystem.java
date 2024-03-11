// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.modules.internal.intake;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.core.Constants;

public class IntakeSubsystem extends SubsystemBase {

    public static IntakeSubsystem instance = null;
    public static PWMVictorSPX intakeMotor;
    public static PWMVictorSPX intake1Motor;

    public IntakeSubsystem() {
        intakeMotor = new PWMVictorSPX(Constants.UpSystemConstants.intakeMotorPWM);
        intake1Motor = new PWMVictorSPX(Constants.UpSystemConstants.intake1MotorPWM);
        //intakeMotor.setInverted(true);
    }

    public static IntakeSubsystem getInstance() {
        if (instance == null) {
            instance = new IntakeSubsystem();
        }
        return instance;
    }

    public final void setMotor(boolean start) {
        if (start) {
            intakeMotor.set(Constants.UpSystemConstants.setIntakeMotorSpeed);
            intake1Motor.set(Constants.UpSystemConstants.setIntakeMotorSpeed);
        } else {
            intakeMotor.stopMotor();
            intake1Motor.stopMotor();
        }
    }

    public final void setReverseMotor(boolean reverse) {
        if (reverse) {
            intakeMotor.set(Constants.UpSystemConstants.setReverseIntakeMotorSpeed);
            intake1Motor.set(Constants.UpSystemConstants.setReverseIntakeMotorSpeed);
        } else {
            intakeMotor.stopMotor();
            intake1Motor.stopMotor();
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
