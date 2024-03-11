// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.modules.internal.shooter.commands;

import frc.robot.core.utils.EnhancedCommand;

public class ShooterSenderCommand extends EnhancedCommand {

    private static ShooterSenderCommand instance = null;

    public ShooterSenderCommand() {
        addRequirements(intakeSubsystem);
    }

    public static ShooterSenderCommand getInstance() {
        if (instance == null) {
            instance = new ShooterSenderCommand();
        }
        return instance;
    }

    public static ShooterSenderCommand forceNewInstance() {
        instance = new ShooterSenderCommand();
        return instance;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        intakeSubsystem.setMotor(true);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setMotor(false);
    }

    @Override
    public boolean isFinished() {
        return !mz80.isSenorDistanceReached();
    }
}
