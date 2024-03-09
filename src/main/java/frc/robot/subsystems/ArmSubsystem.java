// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

    private final CANSparkMax armLmotor;
    private final CANSparkMax armRmotor;
    private final DutyCycleEncoder angleEncoder;

    private static ArmSubsystem instance = null;

    public static ArmSubsystem getInstance() {
        if (instance == null) {
            instance = new ArmSubsystem();
        }
        return instance;
    }

    /**
     * Creates a new ArmSubsystem.
     */
    public ArmSubsystem() {
        armLmotor = new CANSparkMax(Constants.UpSystemConstants.armLmotorId, MotorType.kBrushless);
        armRmotor = new CANSparkMax(Constants.UpSystemConstants.armRmotorId, MotorType.kBrushless);

        armRmotor.setInverted(true);

        armLmotor.setIdleMode(IdleMode.kBrake);
        armRmotor.setIdleMode(IdleMode.kBrake);

        angleEncoder = new DutyCycleEncoder(0);
        angleEncoder.reset();
    }

    public void manuelArmControl(double controller) {
        armLmotor.set(controller * .5);
        armRmotor.set(controller * .5);
    }

    public double getEncoderDegrees() {
        return ((angleEncoder.getAbsolutePosition() * 360) - 39);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Degrees: ", getEncoderDegrees());
    }
}
