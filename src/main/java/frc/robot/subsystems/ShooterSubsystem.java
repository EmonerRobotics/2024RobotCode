// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {

  private static ShooterSubsystem instance = null;
  private CANSparkMax shooterRmotor;
  private CANSparkMax shooterLmotor;

  public static ShooterSubsystem getInstance() {
    if (instance == null) {
      instance = new ShooterSubsystem();
    }
    return instance;
  }

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    shooterRmotor = new CANSparkMax(Constants.UpSystemConstants.rShooterMotorId, MotorType.kBrushless); //ID:10
    shooterLmotor = new CANSparkMax(Constants.UpSystemConstants.lShooterMotorId, MotorType.kBrushless); //ID:11
    
    //Sets motor controller idle settings to brake 
    shooterRmotor.setIdleMode(IdleMode.kBrake);
    shooterLmotor.setIdleMode(IdleMode.kBrake);
  }

  public void setMotors(boolean start){
    if(start){
      shooterRmotor.set(Constants.UpSystemConstants.setShooterMotorSpeeds);
      shooterLmotor.set(Constants.UpSystemConstants.setShooterMotorSpeeds);
    }else{
      shooterRmotor.stopMotor();
      shooterLmotor.stopMotor();
    }
  }

//Puts the data of the color sensor. 
  @Override
  public void periodic() {
  }
}
