// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.modules.internal.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.modules.internal.shooter.ShooterSubsystem;

public class ShooterCommand extends Command {
    private static ShooterCommand instance = null;

    private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();

    public ShooterCommand() {
        addRequirements(shooterSubsystem);
    }

    public static ShooterCommand getInstance() {
        if (instance == null) {
            instance = new ShooterCommand();
        }
        return instance;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        shooterSubsystem.setMotors(true);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("SHOOTER end");
        shooterSubsystem.setMotors(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
