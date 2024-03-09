// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class SafeDrive extends Command {

  public static enum Position{
    ShouldBe,
    Amphi,
    Cover,
  }

  private final ArmSubsystem armSubsystem;
  private final LimelightSubsystem limelightSubsystem;
  private final PIDController pidController;
  private final Position positionControl;

  /** Creates a new AutoArm. */
  //PositionControl positionControl
  public SafeDrive(ArmSubsystem armSubsystem, LimelightSubsystem limelightSubsystem, Position positionControl) {
    this.armSubsystem = armSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    this.pidController = new PIDController(0.06, 0.025, 0.001);
    this.positionControl = positionControl; 
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
    if(positionControl == Position.ShouldBe){
    pidController.setSetpoint(limelightSubsystem.findShooterDegrees());
    double speed = pidController.calculate(armSubsystem.getEncoderDegrees());
    armSubsystem.manuelArmControl(-speed);

    }else if(positionControl == Position.Amphi){
      pidController.setSetpoint(88);
      double speed = 0.5 * pidController.calculate(armSubsystem.getEncoderDegrees());
      armSubsystem.manuelArmControl(-speed);

    }else if(positionControl == Position.Cover){
      pidController.setSetpoint(45);
      double speed = pidController.calculate(armSubsystem.getEncoderDegrees());
      armSubsystem.manuelArmControl(-speed);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.manuelArmControl(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
    /*
    if(positionControl == Position.ShouldBe){
      double error = armSubsystem.getEncoderDegrees() - limelightSubsystem.findShooterDegrees(); 
      SmartDashboard.putNumber("ARM SPEAKER ERROR: ", error);

      if( Math.abs(error) < 1.5){
        System.out.println(" NORMAL ARM BITTI");
        return true;
      }else{
        return false;
      }

    }else if (positionControl == Position.Amphi){
      double error = armSubsystem.getEncoderDegrees() - 88;
      SmartDashboard.putNumber("ARM AMPHI ERROR", error);

      if(Math.abs(error) < 1){
        System.out.println("AMPHI ARM BITTI");
        return true;
      }else{
        return false;
      }
    }else if(positionControl == Position.Cover){
      double error = armSubsystem.getEncoderDegrees() - 45;
      if(Math.abs(error) < 1){
        return true;
      }else{
        return false;
      }
    }else{
      return false;
    }
     */

    //return (Math.abs(error) < 0.5);
    //return false;
  }
}
