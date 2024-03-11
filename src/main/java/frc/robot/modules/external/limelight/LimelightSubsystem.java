// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.modules.external.limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.core.Constants;

import static frc.robot.core.Constants.ModuleConstants.LIMELIGHT_TABLE_NAME;
import static frc.robot.modules.external.limelight.NetworkTableEntryType.*;

public class LimelightSubsystem extends SubsystemBase {
    private static LimelightSubsystem instance = null;

    NetworkTable table = NetworkTableInstance.getDefault().getTable(LIMELIGHT_TABLE_NAME);

    public static LimelightSubsystem getInstance() {
        if (instance == null) {
            instance = new LimelightSubsystem();
        }
        return instance;
    }

    @Override
    public void periodic() {
        getEstimatedDistance();
        SmartDashboard.putBoolean("Found Target", isTargetDetected());
        SmartDashboard.putNumber("ARM Should", findShooterDegrees());
    }

    public double getVerticalTargetOffsetAngle() {
        NetworkTableEntry ty = table.getEntry(TARGET_Y_AXIS.entryCode);
        return ty.getDouble(0.0);
    }

    public double getHorizontalTargetOffsetAngle() {
        NetworkTableEntry tx = table.getEntry(TARGET_X_AXIS.entryCode);
        return tx.getDouble(0.0);
    }

    public double findShooterDegrees() {
        return (0.0095 * Math.pow(getVerticalTargetOffsetAngle(), 2) - 1.0182 * getVerticalTargetOffsetAngle() + 20.195);
    }

    public double getTargetId() {
        NetworkTableEntry targetId = table.getEntry(TARGET_ID.entryCode);
        return targetId.getDouble(0.0);
    }

    public boolean isTargetDetected() {
        NetworkTableEntry targetDetectionValue = table.getEntry(TARGET_DETECTION.entryCode);
        return targetDetectionValue.getFloat(0) != 0.0f;
    }

    public double getEstimatedDistance() {
        getVerticalTargetOffsetAngle();

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 30.5;

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightCentimeter = 49.5;

        // distance from the target to the floor
        double angleToGoalDegrees = limelightMountAngleDegrees + (getVerticalTargetOffsetAngle() - 2);
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance
        double distanceFromLimelightToGoalCentimeter = ((Constants.VisionConstants.ApriltagSpeakerHeight - limelightLensHeightCentimeter) / Math.tan(angleToGoalRadians));
        SmartDashboard.putNumber("Distance", distanceFromLimelightToGoalCentimeter);
        return distanceFromLimelightToGoalCentimeter;
    }
}
