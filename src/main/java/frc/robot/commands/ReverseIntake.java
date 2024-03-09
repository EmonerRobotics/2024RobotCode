// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class ReverseIntake extends Command {

    private static ReverseIntake instance = null;

    private final IntakeSubsystem intakeSubsystem;
    private final boolean reverse;

    public static ReverseIntake getInstance() {
        if (instance == null) {
            instance = new ReverseIntake(
                    IntakeSubsystem.getInstance(),
                    true
            );
        }
        return instance;
    }

    /**
     * Creates a new ReverseIntake.
     */
    public ReverseIntake(IntakeSubsystem intakeSubsystem, boolean reverse) {
        this.intakeSubsystem = intakeSubsystem;
        this.reverse = reverse;
        addRequirements(intakeSubsystem);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (reverse) {
            intakeSubsystem.setReverseMotor(true);
        } else {
            intakeSubsystem.setReverseMotor(false);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setMotor(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
