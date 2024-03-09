package frc.robot.poseestimation;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;

public class PoseEstimation {
    private static PoseEstimation instance = null;

    private final SwerveDrivePoseEstimator poseEstimator;
    private final TimeInterpolatableBuffer<Pose2d> poseHistory = TimeInterpolatableBuffer.createBuffer(1.5);

    public static PoseEstimation getInstance() {
        if (instance == null) {
            instance = new PoseEstimation();
        }
        return instance;
    }

    public PoseEstimation() {
        poseEstimator = new SwerveDrivePoseEstimator(
                DriveConstants.DRIVE_KINEMATICS,
                RobotContainer.drivetrain.getRotation(),
                RobotContainer.drivetrain.getModulePositions(),
                new Pose2d(),
                Constants.DriveConstants.ODOMETRY_STD_DEV,
                VecBuilder.fill(0, 0, 0) // will be overwritten for each measurement
        );
    }

    public void periodic() {
        poseHistory.addSample(
                Timer.getFPGATimestamp(),
                poseEstimator.getEstimatedPosition()
        );

        RobotContainer.field.setRobotPose(getEstimatedPose());
    }

    public void updateOdometry(
            Rotation2d gyro,
            SwerveModulePosition[] modulePositions
    ) {
        poseEstimator.update(gyro, modulePositions);
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(
                RobotContainer.drivetrain.getRotation(),
                RobotContainer.drivetrain.getModulePositions(),
                pose
        );
    }
}