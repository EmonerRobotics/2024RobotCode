// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.modules.internal.intake.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.modules.external.ultrasonic.MZ80;
import frc.robot.modules.internal.intake.IntakeSubsystem;
import frc.robot.modules.internal.intake.IntakeType;

import static frc.robot.core.utils.LoggingUtils.logEvent;
import static frc.robot.core.utils.LoggingUtils.logMessage;

public class IntakeCommand extends Command {
    private static IntakeCommand instance = null;

    private final MZ80 mz80 = MZ80.getInstance();
    private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
    private boolean isCommandEnd = false;
    private IntakeType intakeType;

    public IntakeCommand() {
        addRequirements(intakeSubsystem);
    }

    public static IntakeCommand getInstance(
            IntakeType intakeType
    ) {
        if (instance == null) {
            instance = new IntakeCommand();
        }
        logMessage(intakeType.name());
        instance.setIntakeType(intakeType);
        return instance;
    }

    public static IntakeCommand forceNewInstance(
            IntakeType intakeType
    ) {
        instance = new IntakeCommand();
        instance.setIntakeType(intakeType);
        return instance;
    }

    private void setIntakeType(IntakeType intakeType) {
        this.intakeType = intakeType;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        switch (intakeType) {
            case STANDARD:
                boolean isSensorDistanceReached = mz80.isSenorDistanceReached();

                intakeSubsystem.setMotor(!isSensorDistanceReached);
                isCommandEnd = isSensorDistanceReached;
                break;
            case REVERSE:
                intakeSubsystem.setReverseMotor(true);
                break;
        }

    }

    @Override
    public void end(boolean interrupted) {
        logEvent();
        intakeSubsystem.setMotor(false);
    }

    @Override
    public boolean isFinished() {
        if (intakeType == IntakeType.STANDARD) {
            return isCommandEnd;
        }
        return false;

    }

}
