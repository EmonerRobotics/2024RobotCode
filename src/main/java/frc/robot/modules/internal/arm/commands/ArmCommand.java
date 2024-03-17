// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.modules.internal.arm.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.core.enums.PositionType;
import frc.robot.modules.external.limelight.LimelightSubsystem;
import frc.robot.modules.external.ultrasonic.MZ80;
import frc.robot.modules.internal.arm.ArmSubsystem;

import java.util.Objects;

public class ArmCommand extends Command {
    private static ArmCommand instance = null;

    private ArmCommandCallback callback;

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
       // logMessage("arm initialize");
        setArmLocked();
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
        boolean commandShouldFinish = !armLocked || !MZ80.getInstance().isSenorDistanceReached();

        double errorMargin = armSubsystem.getEncoderDegrees() - positionType.positionDegree;


        switch (positionType) {
            case TARGET:
                errorMargin = limelightSubsystem.findShooterDegrees() - armSubsystem.getEncoderDegrees();
                SmartDashboard.putNumber("ARM SPEAKER ERROR: ", errorMargin);

                if(errorMargin < 0.5){
                   // logMessage("ARM COMMAND SHOULD END");
                    callback.shoot();
                }

                break;
            case AMPHI:
                SmartDashboard.putNumber("ARM AMPHI ERROR", errorMargin);
                break;

        }

        return commandShouldFinish;
    }

    @Override
    public void end(boolean interrupted) {

        setArmLocked();

        if (positionType == PositionType.GROUND) {
            System.out.println("GROUND end");
        }
        else {
            System.out.println("END");
            armSubsystem.manuelArmControl(0);
        }

    }


}
