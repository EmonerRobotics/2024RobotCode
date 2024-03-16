// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.modules.internal.arm.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.core.enums.PositionType;
import frc.robot.modules.external.limelight.LimelightSubsystem;
import frc.robot.modules.internal.arm.ArmSubsystem;

import java.util.Objects;

import static frc.robot.core.utils.LoggingUtils.logEvent;
import static frc.robot.core.utils.LoggingUtils.logMessage;

public class ArmCommand extends Command {
    private static ArmCommand instance = null;

    private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    private final LimelightSubsystem limelightSubsystem = LimelightSubsystem.getInstance();
    private final PIDController pidController;
    private PositionType positionType;
    private boolean armLocked = false;

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
        System.out.println("furkan 1");
        logMessage("test test 1");
        return instance;
    }

    private void setArmLocked(){
        armLocked = !armLocked;
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
        logMessage("arm initialize");
        setArmLocked();
        logMessage("isArmLocked: " + armLocked);
        pidController.reset();
    }

    @Override
    public void execute() {
        logMessage("arm execute");
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
        boolean commandShouldFinish = !armLocked;

        double errorMargin = armSubsystem.getEncoderDegrees() - positionType.positionDegree;


        switch (positionType) {
            case TARGET:
                errorMargin = armSubsystem.getEncoderDegrees() - limelightSubsystem.findShooterDegrees();
                SmartDashboard.putNumber("ARM SPEAKER ERROR: ", errorMargin);
                break;
            case AMPHI:
                SmartDashboard.putNumber("ARM AMPHI ERROR", errorMargin);

                break;

        }

        return commandShouldFinish;
    }

    @Override
    public void end(boolean interrupted) {
        logMessage("arm end");

        setArmLocked();
        logMessage("is arm locked in end: " + armLocked);

        if (positionType == PositionType.GROUND) {
            System.out.println("GROUND end");
        } else {
            armSubsystem.manuelArmControl(0);
        }

    }


}
