// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

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

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        armLockSubsystem.setReverseMotor(true);
    }

    @Override
    public void end(boolean interrupted) {
        armLockSubsystem.setLockMotor(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
