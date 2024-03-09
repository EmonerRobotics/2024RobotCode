// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimelightSubsystem extends SubsystemBase {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    private static LimelightSubsystem instance = null;

    public static LimelightSubsystem getInstance() {
        if (instance == null) {
            instance = new LimelightSubsystem();
        }
        return instance;
    }

    @Override
    public void periodic() {
        EstimatedDistance();
        SmartDashboard.putBoolean("Found Target", hasTargets());
        SmartDashboard.putNumber("ARM Should", findShooterDegrees());
    }

    public double getTy() {
        NetworkTableEntry ty = table.getEntry("ty");
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);
        return targetOffsetAngle_Vertical;
    }

    public double getAim() {
        NetworkTableEntry tx = table.getEntry("tx");
        double headingError = tx.getDouble(0.0);
        return headingError;
    }

    public double EstimatedDistance() {
        getTy();

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 30.5;

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightCentimeter = 49.5;

        // distance from the target to the floor
        double angleToGoalDegrees = limelightMountAngleDegrees + (getTy() - 2);
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance
        double distanceFromLimelightToGoalCentimeter = ((Constants.VisionConstants.ApriltagSpeakerHeight - limelightLensHeightCentimeter) / Math.tan(angleToGoalRadians));
        SmartDashboard.putNumber("Distance", distanceFromLimelightToGoalCentimeter);
        return distanceFromLimelightToGoalCentimeter;
    }

    //return (0.0095 * Math.pow(getTy(), 2) - 1.0182 * getTy() + 20.195);
    public double findShooterDegrees() {

        return (0.0095 * Math.pow(getTy(), 2) - 1.0182 * getTy() + 24.5); //24.5
    }

    public double getId() {
        NetworkTableEntry tid = table.getEntry("tid");
        double tId = tid.getDouble(0.0);
        return tId;
    }

    public boolean hasTargets() {
        NetworkTableEntry tv = table.getEntry("tv");
        float isTrue = tv.getFloat(0);
        return (isTrue == 0.0f) ? false : true;
    }
}
