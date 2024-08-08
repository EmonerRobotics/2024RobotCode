// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class SlowArmDown extends Command {

  public static enum PositionController{
    ShouldBe,
    Zero
  }

  private final ArmSubsystem armSubsystem;
  private final PIDController pidController;
  private final PositionController positionController;

  /** Creates a new AutoArm. */
  //PositionControl positionControl
  public SlowArmDown(ArmSubsystem armSubsystem,  PositionController positionController) {
    this.armSubsystem = armSubsystem;
    this.pidController = new PIDController(0.02, 0.001, 0);
    this.positionController = positionController; 
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(positionController == PositionController.ShouldBe){

    pidController.setSetpoint(11); //limelightSubsystem.findShooterDegrees()
    double speed = pidController.calculate(armSubsystem.getEncoderDegrees());
    armSubsystem.manuelArmControl(-speed);

    }else if(positionController == PositionController.Zero){
    pidController.setSetpoint(1.5); //limelightSubsystem.findShooterDegrees()
    double speed = pidController.calculate(armSubsystem.getEncoderDegrees());
    armSubsystem.manuelArmControl(-speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("SLOW ARM BITTI");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   
    if(positionController == PositionController.ShouldBe){
    double error = armSubsystem.getEncoderDegrees() - 11; 
    
    // limelightSubsystem.findShooterDegrees(); 
    System.out.println("ARM ERROR: " + Math.abs(error));
    if( Math.abs(error) < 1.5){
      System.out.println("SLOW ARM BITTI");
      return true;
    }else{
      return false;
    }
  }else if(positionController == PositionController.Zero){
     double error = armSubsystem.getEncoderDegrees() - 1.5; 
      System.out.println("ARM ERROR: " + Math.abs(error));
    if( Math.abs(error) < 1.5){
      System.out.println("SLOW ARM BITTI");
      return true;
    }else{
      return false;
    }
     }
     else{
      return false;
     }
    //return (Math.abs(error) < 0.5);
    //return false;
  }
}
