// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {

  private final ShooterSubsystem shooterSubsystem;
  private final boolean start;

  /** Creates a new ShooterCommand. */
  public ShooterCommand(ShooterSubsystem shooterSubsystem, boolean start) {
    this.shooterSubsystem = shooterSubsystem;
    this.start = start;
    addRequirements(shooterSubsystem);
  } 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.setMotors(start);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    System.out.println("SHOOTER END");
    shooterSubsystem.setMotors(!start);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return start ? false : true;
    if(!start){
      return true;
    }else{
      return false;
    }
  }
}
