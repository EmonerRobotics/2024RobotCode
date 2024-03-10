// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.enums.PositionType;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

import java.util.Objects;

public class ArmCommand extends Command {
    private static ArmCommand instance = null;

    private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    private final LimelightSubsystem limelightSubsystem = LimelightSubsystem.getInstance();
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
        armSubsystem.manuelArmControl( -1 * positionType.speedMultiplier * armControlOutput);

    }

    @Override
    public void end(boolean interrupted) {
        if (positionType == PositionType.GROUND) {
            System.out.println("GROUND end");
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
                System.out.println("ARM TARGET finished");
                errorMargin = armSubsystem.getEncoderDegrees() - limelightSubsystem.findShooterDegrees();
                SmartDashboard.putNumber("ARM SPEAKER ERROR: ", errorMargin);
                break;
            case AMPHI:
                System.out.println("ARM AMPHI finished");
                SmartDashboard.putNumber("ARM AMPHI ERROR", errorMargin);
                threshold = 1;
                break;
            case AUTO:
                System.out.println("ARM AUTO finished");
                break;
            case GROUND:
                System.out.println("ARM GROUND finished");
                break;

        }

        if (Math.abs(errorMargin) < threshold) {
            System.out.println(positionType.name() + " finished");
            return true;
        } else {
            return false;
        }
    }

}
