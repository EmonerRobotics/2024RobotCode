// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {
    private static ShooterCommand instance = null;

    private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    private final boolean start;

    public ShooterCommand() {
        //TODO: start usage is suspecious
        this.start = true;
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
        shooterSubsystem.setMotors(start);
    }

    @Override
    public void end(boolean interrupted) {

        System.out.println("SHOOTER END");
        shooterSubsystem.setMotors(!start);
    }

    @Override
    public boolean isFinished() {
        if (!start) {
            return true;
        } else {
            return false;
        }
    }
}
