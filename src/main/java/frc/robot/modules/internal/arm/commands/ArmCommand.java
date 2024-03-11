// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.modules.internal.arm.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.core.enums.PositionType;
import frc.robot.core.utils.EnhancedCommand;
import frc.robot.modules.external.limelight.LimelightSubsystem;
import frc.robot.modules.internal.arm.ArmSubsystem;

import java.util.Objects;

import static frc.robot.core.utils.LoggingUtils.logMessage;

public class ArmCommand extends EnhancedCommand {
    private static ArmCommand instance = null;

    private final PIDController pidController;
    private PositionType positionType;

    private ArmCommand() {
        this.pidController = new PIDController(0, 0, 0);
        addRequirements(armSubsystem);
    }

    public static ArmCommand getInstance(
            PositionType positionType
    ) {
        if (instance == null) {
            instance = new ArmCommand();
        }
        instance.setPIDController(positionType);
        return instance;
    }

    public static ArmCommand forceNewInstance(PositionType positionType) {
        instance = new ArmCommand();
        instance.setPIDController(positionType);
        return instance;
    }

    private void setPIDController(PositionType positionType) {
        if (this.positionType != positionType) {
            if (Objects.requireNonNull(positionType) == PositionType.GROUND) {
                this.pidController.setPID(0.02, 0.001, 0);
            } else {
                this.pidController.setPID(0.06, 0.025, 0.001);
            }
            this.positionType = positionType;
        }
    }

    @Override
    public void initialize() {
        pidController.reset();
    }

    @Override
    public void execute() {
        double armControlOutput = pidController.calculate(armSubsystem.getEncoderDegrees());

        if (positionType == PositionType.TARGET) {
            pidController.setSetpoint(limelightSubsystem.findShooterDegrees());
        } else {
            pidController.setSetpoint(positionType.positionDegree);
        }
        armSubsystem.manuelArmControl(-1 * positionType.speedMultiplier * armControlOutput);

    }

    @Override
    public void end(boolean interrupted) {
        if (positionType == PositionType.GROUND) {
            logMessage(positionType.name() + " end");
        } else {
            armSubsystem.manuelArmControl(0);
        }

    }

    @Override
    public boolean isFinished() {
        double errorMargin = armSubsystem.getEncoderDegrees() - positionType.positionDegree;
        double threshold = 1.5;

        switch (positionType) {
            case TARGET:
                errorMargin = armSubsystem.getEncoderDegrees() - limelightSubsystem.findShooterDegrees();
                SmartDashboard.putNumber("ARM SPEAKER ERROR: ", errorMargin);
                break;
            case AMPHI:
                SmartDashboard.putNumber("ARM AMPHI ERROR", errorMargin);
                threshold = 1;
                break;

        }

        if (Math.abs(errorMargin) < threshold) {
            logMessage(positionType.name() + " finished");
            return true;
        } else {
            return false;
        }
    }

}
