// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.modules.internal.arm.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.modules.internal.arm.ArmSubsystem;

import java.util.function.Supplier;

public class ManuelArmCommand extends Command {

    private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    private final Supplier<Double> controller;

    public ManuelArmCommand(Supplier<Double> controller) {
        this.controller = controller;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double controllers = controller.get();
        armSubsystem.manuelArmControl(controllers);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.manuelArmControl(0);
    }

    @Override
    public boolean isFinished() {
        return false;

    }
}
