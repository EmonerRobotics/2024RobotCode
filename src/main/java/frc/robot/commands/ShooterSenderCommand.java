// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MZ80;

public class ShooterSenderCommand extends Command {
  
  private final MZ80 mz80;
  private final IntakeSubsystem intakeSubsystem;
  private boolean start;
  

  /** Creates a new IntakeCommand. */
  public ShooterSenderCommand(IntakeSubsystem intakeSubsystem, MZ80 mz80, boolean start) {
    this.intakeSubsystem = intakeSubsystem;
    this.mz80 = mz80;
    this.start = start;
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(start){
      intakeSubsystem.setMotor(true);
    }else{
      intakeSubsystem.setMotor(false);
    }

    /*

    if(start && mz80.sensorget() == true){
      intakeSubsystem.setMotor(true);
    }else{
      intakeSubsystem.setMotor(false);
    }  */

    /*
    
    if(start){
      intakeSubsystem.setMotor(start);
      if(colorDetection.getColor() == "Orange"){
        intakeSubsystem.setMotor(!start);
        stop = true;
        start = false;
      }else{
        intakeSubsystem.setMotor(start);
        stop = false;
      }
    }else{
      intakeSubsystem.setMotor(!start);
      stop =  true;
      start = false;
       */
    }

    //SmartDashboard.putBoolean("START", start);
    //SmartDashboard.putBoolean("STOP", stop);
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setMotor(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if(mz80.sensorget()){
      return false;
    }else{
      return true;
    }

    /* 
    if(mz80.sensorget()==false){
      return true;
    }else{
      return false;*/
    //}
    //return start ? false : true;
    
    /*
    if(stop){
      return true;
    }else{
      return false;
    } */
  }
}
