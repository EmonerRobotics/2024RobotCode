// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmLockSubsystem extends SubsystemBase {
  private PWMVictorSPX armLockMotor;
  /** Creates a new ArmLockSubsystem. */
  public ArmLockSubsystem() {
    armLockMotor = new PWMVictorSPX(0);
  }

  public void setLockMotor(boolean start){
    if(start){
      armLockMotor.set(1);
  }else{
    armLockMotor.stopMotor();
  }
}

public void setReverseMotor(boolean Reverse){
   if(Reverse){
      armLockMotor.set(-1);
  }else{
    armLockMotor.stopMotor();
  }
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
