// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.modules.internal.arm.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.core.utils.EnhancedCommand;
import frc.robot.modules.internal.arm.ArmLockSubsystem;

public class ArmLockCommand extends EnhancedCommand {
    private static ArmLockCommand instance = null;

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

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        armLockSubsystem.setLockMotor(true);
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
