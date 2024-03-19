// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.modules.internal.arm.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autonomous.CenterToTarget;
import frc.robot.core.enums.PositionType;
import frc.robot.modules.external.limelight.LimelightSubsystem;
import frc.robot.modules.external.ultrasonic.MZ80;
import frc.robot.modules.internal.arm.ArmSubsystem;

import java.util.Objects;

import static frc.robot.core.utils.LoggingUtils.logEvent;

public class ArmCommand extends Command {
    private static ArmCommand instance = null;
    private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    private final LimelightSubsystem limelightSubsystem = LimelightSubsystem.getInstance();
    private final PIDController pidController;
    private ArmCommandCallback callback;
    private PositionType positionType;
    private boolean armLocked = false;
    private boolean isFirstAttemptToShoot = true;

    private ArmCommand() {
        this.pidController = new PIDController(0, 0, 0);
        addRequirements(armSubsystem);
    }

    public static ArmCommand getInstance(
            PositionType positionType,
            ArmCommandCallback callback
    ) {
        if (instance == null) {
            instance = new ArmCommand();
        }
        instance.setPIDController(positionType);
        instance.callback = callback;
        return instance;
    }

    public static ArmCommand forceNewInstance(
            PositionType positionType,
            ArmCommandCallback callback
    ) {
        instance = new ArmCommand();
        instance.setPIDController(positionType);
        instance.callback = callback;
        logEvent();
        return instance;
    }

    private void setArmLocked(boolean value) {
        armLocked = value;
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
        logEvent();
        setArmLocked(true);
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
    public boolean isFinished() {
        boolean targetCommandShouldFinish = false;
        boolean amphiCommandShouldFinish = false;

        double errorMargin;

        switch (positionType) {
            case TARGET:
                targetCommandShouldFinish = !MZ80.getInstance().isSenorDistanceReached();

                errorMargin = limelightSubsystem.findShooterDegrees() - armSubsystem.getEncoderDegrees();
                SmartDashboard.putNumber("ARM Shooter Degree: ", limelightSubsystem.findShooterDegrees());
                SmartDashboard.putNumber("ARM Encoder Degree: ", armSubsystem.getEncoderDegrees());
                SmartDashboard.putNumber("ARM(TARGET) Error Margin: ", errorMargin);

                if (errorMargin <= positionType.threasold) {
                    if (isFirstAttemptToShoot && !CenterToTarget.getInstance().getIsCenterToTargetActive()) {
                        callback.shoot();
                        isFirstAttemptToShoot = false;
                    }

                }

                if (targetCommandShouldFinish) {
                    isFirstAttemptToShoot = true;
                }

                break;
            case AMPHI:
                errorMargin = Math.abs(armSubsystem.getEncoderDegrees() - positionType.positionDegree);
                SmartDashboard.putNumber("ARM(AMPHI) Encoder Margin: ", armSubsystem.getEncoderDegrees());
                SmartDashboard.putNumber("ARM(AMPHI) position Margin: ", positionType.positionDegree);
                SmartDashboard.putNumber("ARM(AMPHI) Error Margin: ", errorMargin);
                if (errorMargin <= positionType.threasold) {
                    amphiCommandShouldFinish = true;
                }
                break;

            default:
                return true;

        }

        return targetCommandShouldFinish || amphiCommandShouldFinish;
    }

    @Override
    public void end(boolean interrupted) {
        logEvent();
        setArmLocked(false);

        if (positionType == PositionType.GROUND) {
        } else {
            armSubsystem.manuelArmControl(0);
        }

    }


}
