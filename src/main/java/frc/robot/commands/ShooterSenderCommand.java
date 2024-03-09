// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MZ80;

public class ShooterSenderCommand extends Command {

    private static ShooterSenderCommand instance = null;

    private final MZ80 mz80 = MZ80.getInstance();
    private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();


    public static ShooterSenderCommand getInstance() {
        if (instance == null) {
            instance = new ShooterSenderCommand();
        }
        return instance;
    }

    /**
     * Creates a new IntakeCommand.
     */
    public ShooterSenderCommand() {
        addRequirements(intakeSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intakeSubsystem.setMotor(true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setMotor(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return !mz80.isSenorDistanceReached();
    }
}
