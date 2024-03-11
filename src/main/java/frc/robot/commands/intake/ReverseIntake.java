// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class ReverseIntake extends Command {
    private static ReverseIntake instance = null;

    //TODO: if same reference is used, it will be better or not
    private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();

    public ReverseIntake() {
        addRequirements(intakeSubsystem);
    }

    public static ReverseIntake getInstance() {
        if (instance == null) {
            instance = new ReverseIntake();
        }
        return instance;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        intakeSubsystem.setReverseMotor(true);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setMotor(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
