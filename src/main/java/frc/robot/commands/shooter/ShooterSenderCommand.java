// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MZ80;

public class ShooterSenderCommand extends Command {

    private static ShooterSenderCommand instance = null;

    private final MZ80 mz80 = MZ80.getInstance();
    private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();

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
