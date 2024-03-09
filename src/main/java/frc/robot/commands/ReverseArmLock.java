// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmLockSubsystem;

public class ReverseArmLock extends Command {

    private static ReverseArmLock instance = null;

    public ArmLockSubsystem armLockSubsystem = ArmLockSubsystem.getInstance();

    public ReverseArmLock() {
        addRequirements(armLockSubsystem);
    }

    public static ReverseArmLock getInstance() {
        if (instance == null) {
            instance = new ReverseArmLock();
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
        armLockSubsystem.setReverseMotor(true);
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
