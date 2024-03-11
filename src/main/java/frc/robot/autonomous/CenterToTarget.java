// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.core.utils.EnhancedCommand;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.modules.external.limelight.LimelightSubsystem;
import frc.robot.pose_estimation.PoseEstimation;

public class CenterToTarget extends EnhancedCommand {
    private static CenterToTarget instance = null;

    public static final int HORIZONTAL_MAX_ERROR_ANGLE = 1;
    private final PIDController pidController;
    private double speedY;

    public CenterToTarget() {
        this.pidController = new PIDController(0.04, 0.02, 0);
        addRequirements(limelightSubsystem);
    }

    public static CenterToTarget getInstance() {
        if (instance == null) {
            instance = new CenterToTarget();
        }
        return instance;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (limelightSubsystem.getTargetId() == 3 || limelightSubsystem.getTargetId() == 7) {

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

    @Override
    public void end(boolean interrupted) {
        ChassisSpeeds fieldRelSpeeds = new ChassisSpeeds(0, 0, 0);
        ChassisSpeeds robotRelSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                fieldRelSpeeds,
                poseEstimation.getEstimatedPose().getRotation()
        );
        drivetrain.drive(robotRelSpeeds);

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
