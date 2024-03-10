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
    private Rotation2d fieldOrientationZeroOffset = new Rotation2d();
    private double speedY;


    public CenterToTarget() {
        this.pidController = new PIDController(0.03, 0.02, 0);
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
        if (limelightSubsystem.getTargetId() == 3 || limelightSubsystem.getTargetId() == 7) {
            if (limelightSubsystem.isTargetDetected()) {
                pidController.setSetpoint(0);
                speedY = pidController.calculate(limelightSubsystem.getHorizontalTargetOffsetAngle());
                ChassisSpeeds fieldRelSpeeds = new ChassisSpeeds(0, 0, speedY);
                System.out.println("CENTER SPEEDY:" + speedY);
                ChassisSpeeds robotRelSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        fieldRelSpeeds,
                        poseEstimation.getEstimatedPose().getRotation()
                                .minus(AllianceUtils.getFieldOrientationZero().plus(fieldOrientationZeroOffset))
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
        ChassisSpeeds robotRelSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelSpeeds, poseEstimation.getEstimatedPose().getRotation().minus(AllianceUtils.getFieldOrientationZero().plus(fieldOrientationZeroOffset)));
        drivetrain.drive(robotRelSpeeds);

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (Math.abs(limelightSubsystem.getHorizontalTargetOffsetAngle()) < 2) {
            System.out.println("CENTER END");
            speedY = 0;
            return true;
        } else {
            return false;

        }
    }
}
