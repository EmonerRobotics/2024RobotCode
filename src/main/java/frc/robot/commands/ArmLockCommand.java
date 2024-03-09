// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmLockSubsystem;

public class ArmLockCommand extends Command {

    private static ArmLockCommand instance = null;

    public ArmLockSubsystem armLockSubsystem = ArmLockSubsystem.getInstance();

    public ArmLockCommand() {
        addRequirements(armLockSubsystem);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    public static ArmLockCommand getInstance() {
        if (instance == null) {
            instance = new ArmLockCommand();
        }
        return instance;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        armLockSubsystem.setLockMotor(true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        armLockSubsystem.setLockMotor(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
