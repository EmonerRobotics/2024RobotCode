// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    private static ArmSubsystem instance = null;

    private final CANSparkMax armLmotor = new CANSparkMax(
            Constants.UpSystemConstants.armLmotorId,
            MotorType.kBrushless
    );
    private final CANSparkMax armRmotor = new CANSparkMax(
            Constants.UpSystemConstants.armRmotorId,
            MotorType.kBrushless
    );
    private final DutyCycleEncoder angleEncoder = new DutyCycleEncoder(0);

    public ArmSubsystem() {
        armRmotor.setInverted(true);

        setIdleModesOfArmMotorsAsImmediateBrake();
        resetEncoderDistanceToZero();
    }

    public static ArmSubsystem getInstance() {
        if (instance == null) {
            instance = new ArmSubsystem();
        }
        return instance;
    }

    public void manuelArmControl(double controller) {
        armLmotor.set(controller * .5);
        armRmotor.set(controller * .5);
    }

    public double getEncoderDegrees() {
        return ((angleEncoder.getAbsolutePosition() * 360) - 39);
    }

    private void setIdleModesOfArmMotorsAsImmediateBrake() {
        armLmotor.setIdleMode(IdleMode.kBrake);
        armRmotor.setIdleMode(IdleMode.kBrake);
    }

    private void resetEncoderDistanceToZero() {
        angleEncoder.reset();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Degrees: ", getEncoderDegrees());
    }

}
