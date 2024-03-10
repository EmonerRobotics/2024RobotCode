// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.AllianceUtils;

public class CenterToTarget extends Command {

    private final LimelightSubsystem limelightSubsystem = LimelightSubsystem.getInstance();
    private final PIDController pidController;
    private final Drivetrain drivetrain = Drivetrain.getInstance();
    private final PoseEstimation poseEstimation = PoseEstimation.getInstance();
    private double speedY;


    public CenterToTarget() {
        this.pidController = new PIDController(0.04, 0.02, 0);
        addRequirements(limelightSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("CENTER BASLADI");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.println("CenterToTarget: executing");

        if (limelightSubsystem.getTargetId() == 3 || limelightSubsystem.getTargetId() == 7) {
            System.out.println("CenterToTarget: target id 3 or 7");
            if (limelightSubsystem.isTargetDetected()) {
                System.out.println("CenterToTarget: target detected");

                pidController.setSetpoint(0);

                speedY = pidController.calculate(limelightSubsystem.getHorizontalTargetOffsetAngle());
                System.out.println("CENTER SPEED-Y:" + speedY);

                ChassisSpeeds fieldRelSpeeds = new ChassisSpeeds(0, 0, speedY);
                ChassisSpeeds robotRelSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        fieldRelSpeeds,
                        poseEstimation.getEstimatedPose().getRotation()
                );

                drivetrain.drive(robotRelSpeeds);
            } else {
                System.out.println("NO TARGET");
            }
        } else {
            System.out.println("NO TARGET");
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        ChassisSpeeds fieldRelSpeeds = new ChassisSpeeds(0, 0, 0);
        ChassisSpeeds robotRelSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                fieldRelSpeeds,
                poseEstimation.getEstimatedPose().getRotation()
        );
        drivetrain.drive(robotRelSpeeds);

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        System.out.println("tx" + Math.abs(limelightSubsystem.getHorizontalTargetOffsetAngle()));
        System.out.println("tx bool" + String.valueOf(Math.abs(limelightSubsystem.getHorizontalTargetOffsetAngle()) < 2));

        if(!limelightSubsystem.isTargetDetected()) {
            return false;
        }

        if (Math.abs(limelightSubsystem.getHorizontalTargetOffsetAngle()) < 1) {
            System.out.println("CENTER end");
            return true;
        } else {
            return false;

        }
    }
}
