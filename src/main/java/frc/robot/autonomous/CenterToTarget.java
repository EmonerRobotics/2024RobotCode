// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.modules.internal.drivetrain.DriveSubsystem;
import frc.robot.modules.external.limelight.LimelightSubsystem;

public class CenterToTarget extends Command {

    public static CenterToTarget instance = null;

    public static final double HORIZONTAL_MAX_ERROR_ANGLE = 0.5;
    private final LimelightSubsystem limelightSubsystem = LimelightSubsystem.getInstance();
    private final PIDController pidController;
    private final DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();
    private double speedY;

    public static CenterToTarget getInstance() {
        if (instance == null) {
            instance = new CenterToTarget();
        }
        return instance;
    }

    public CenterToTarget() {
        pidController = new PIDController(
                0.03,
                0.03,
                0.01
        );
        //  pidController.setIZone(0.01);
        pidController.setSetpoint(0);
        addRequirements(limelightSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("CENTER BASLADI");
    }

    @Override
    public void execute() {
        System.out.println(limelightSubsystem.getEstimatedDistance());

        if (limelightSubsystem.getHorizontalTargetOffsetAngle() != 0) {
            speedY = pidController.calculate(
                    limelightSubsystem.getHorizontalTargetOffsetAngle()
            );
        }

        double actualSpeed;

        if (limelightSubsystem.getEstimatedDistance() > 222) {
            actualSpeed = speedY / 9;
        } else {
            actualSpeed = speedY / 5;
        }


        System.out.println("CENTER SPEED-Y:" + actualSpeed);
        System.out.println("limeli:" + limelightSubsystem.getHorizontalTargetOffsetAngle());
                driveSubsystem.drive(
                        0,
                        0,
                        actualSpeed,
                        true,
                        false
                );
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(
                0,
                0,
                0,
                true,
                false

        );

    }

    @Override
    public boolean isFinished() {
        boolean isTargetDetected = limelightSubsystem.isTargetDetected();
        boolean isHorizontalTargetOffsetAngleErrorReached = Math.abs(limelightSubsystem.getHorizontalTargetOffsetAngle()) < HORIZONTAL_MAX_ERROR_ANGLE;

        if (isHorizontalTargetOffsetAngleErrorReached && isTargetDetected) {
            System.out.println("CENTER end");
            return true;
        }

        return false;
    }
}
