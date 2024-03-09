// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MZ80;

public class IntakeCommand extends Command {

    private static IntakeCommand instance = null;
    private final MZ80 mz80;
    private final IntakeSubsystem intakeSubsystem;
    private boolean start;
    private boolean isCommandEnd = false;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, MZ80 mz80, boolean start) {
        this.intakeSubsystem = intakeSubsystem;
        this.mz80 = mz80;
        this.start = start;
        addRequirements(intakeSubsystem);
    }

    public static IntakeCommand getInstance(
    ) {
        if (instance == null) {
            instance = new IntakeCommand(
                    IntakeSubsystem.getInstance(),
                    MZ80.getInstance(),
                    true
            );
        }
        return instance;
    }

   
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        boolean isSensorDistanceReached = mz80.isSenorDistanceReached();
        if (start) {
          intakeSubsystem.setMotor(!isSensorDistanceReached);
          isCommandEnd = isSensorDistanceReached;
        } else {
            intakeSubsystem.setMotor(isSensorDistanceReached);
        }

    }
 
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setMotor(false);
    }

    @Override
    public boolean isFinished() {
        System.out.println("-----ultrasonic sensor");
        System.out.println(String.valueOf(mz80.isSenorDistanceReached()));
        return isCommandEnd;
  
    }
}
