// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.modules.internal.shooter.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.modules.external.ultrasonic.MZ80;
import frc.robot.modules.internal.intake.IntakeSubsystem;

import static frc.robot.core.utils.LoggingUtils.logEvent;
import static frc.robot.core.utils.LoggingUtils.logMessage;

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
        logMessage("*********************NEW INSTANCE***************************************");
        instance = new ShooterSenderCommand();
        return instance;
    }

    @Override
    public void initialize() {
        logMessage("********************** INIT **************************************");
        logEvent();
    }

    @Override
    public void execute() {
        logMessage("********************** EXEC **************************************");
        intakeSubsystem.setMotor(true);
    }

    @Override
    public void end(boolean interrupted) {
        logMessage("********************** END **************************************");
        logEvent();
        intakeSubsystem.setMotor(false);
    }

    @Override
    public boolean isFinished() {
        return !mz80.isSenorDistanceReached();
    }
}
