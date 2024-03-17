package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autonomous.enums.CenteringStartPosition;
import frc.robot.modules.internal.drivetrain.DriveSubsystem;
import frc.robot.modules.external.limelight.LimelightSubsystem;

import static frc.robot.core.utils.LoggingUtils.logEvent;
import static frc.robot.core.utils.LoggingUtils.logMessage;

public class CenterToTarget extends Command {

    public static CenterToTarget instance = null;

    public static final double HORIZONTAL_MAX_ERROR_ANGLE = 0.9;
    public static final double MINIMUM_SPEED_THRESHOLD = 0.045;
    public static final double MINIMUM_SPEED = 0.06;
    public static final double SPEED_DIVIDER = 3.2;

    private final LimelightSubsystem limelightSubsystem = LimelightSubsystem.getInstance();
    private final PIDController pidController;
    private final DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();
    private double centeringSpeed;
    private double cacheLimelightAngle;

    public static CenterToTarget getInstance() {
        if (instance == null) {
            instance = new CenterToTarget();
        }
        return instance;
    }

    public CenterToTarget() {
        pidController = new PIDController(
                0.042,
                0.0,
                0.0
        );
        pidController.setSetpoint(0);

        addRequirements(limelightSubsystem);
    }

    private double getCurrentLimelightAngle() {
        return limelightSubsystem.getHorizontalTargetOffsetAngle();
    }

    private void setCenteringSpeed(double centeringSpeed) {
        this.centeringSpeed = centeringSpeed;
    }

    private void setLimelightCacheWithNewAngle(double currentLimelightAngle) {
        cacheLimelightAngle = currentLimelightAngle;
    }

    private boolean isAnomalyDetectedForLeft(double currentLimelightAngle) {
        return currentLimelightAngle > cacheLimelightAngle;
    }

    private boolean isAnomalyDetectedForRight(double currentLimelightAngle) {
        return currentLimelightAngle < cacheLimelightAngle;
    }

    private boolean isTargetDetected() {
        return limelightSubsystem.isTargetDetected();
    }

    private double calculateCenteringSpeedWithPid(double angle) {
        return pidController.calculate(
                angle
        );
    }

    public CenteringStartPosition getStartingPosition() {
        if (centeringSpeed < 0) {
            return CenteringStartPosition.LEFT;
        } else {
            return CenteringStartPosition.RIGHT;
        }
    }


    private double slowDownCenteringSpeed(double speedDivider) {
        return centeringSpeed / speedDivider;
    }

    private double determineCenteringSpeedLowLimit() {
        switch (getStartingPosition()) {
            case LEFT:
                if (centeringSpeed > -MINIMUM_SPEED_THRESHOLD) {
                    return -MINIMUM_SPEED;
                }

            case RIGHT:
                if (centeringSpeed < MINIMUM_SPEED_THRESHOLD) {
                    return MINIMUM_SPEED;
                }
            default:
                return centeringSpeed;
        }


    }

    private void updateCenteringSpeedForAnomalies(
            double currentLimelightAngle
    ) {
        double newSpeed;
        switch (getStartingPosition()) {
            case LEFT:
                if (!isAnomalyDetectedForLeft(currentLimelightAngle)) {
                    newSpeed = calculateCenteringSpeedWithPid(currentLimelightAngle);
                    setCenteringSpeed(newSpeed);
                    setLimelightCacheWithNewAngle(currentLimelightAngle);
                } else {
                    newSpeed = calculateCenteringSpeedWithPid(cacheLimelightAngle);
                    setCenteringSpeed(newSpeed);
                }
                break;
            case RIGHT:
                if (!isAnomalyDetectedForRight(currentLimelightAngle)) {
                    newSpeed = calculateCenteringSpeedWithPid(currentLimelightAngle);
                    setCenteringSpeed(newSpeed);
                    setLimelightCacheWithNewAngle(currentLimelightAngle);
                } else {
                    newSpeed = calculateCenteringSpeedWithPid(cacheLimelightAngle);
                    setCenteringSpeed(newSpeed);
                }
                break;
        }
    }

    @Override
    public void initialize() {
        logEvent();
        cacheLimelightAngle = limelightSubsystem.getHorizontalTargetOffsetAngle();
    }

    @Override
    public void execute() {
        if (isTargetDetected()) {
            updateCenteringSpeedForAnomalies(
                    getCurrentLimelightAngle()
            );

        }

        setCenteringSpeed(slowDownCenteringSpeed(SPEED_DIVIDER));

        setCenteringSpeed(determineCenteringSpeedLowLimit());

        driveSubsystem.drive(
                0,
                0,
                centeringSpeed,
                true,
                false
        );

    }

    @Override
    public void end(boolean interrupted) {
        logEvent();
        driveSubsystem.drive(
                0,
                0,
                0,
                true,
                false

        );
    }

    private boolean targetDetected = false;
    private long lastDetectionTime = System.currentTimeMillis();

    @Override
    public boolean isFinished() {
        boolean isHorizontalTargetOffsetAngleErrorReached =
                Math.abs(limelightSubsystem.getHorizontalTargetOffsetAngle()) < HORIZONTAL_MAX_ERROR_ANGLE;

        if (!isTargetDetected()) {
            long currentTime = System.currentTimeMillis();
            long timeSinceLastDetection = currentTime - lastDetectionTime;

            if (timeSinceLastDetection >= 2000) {
                logMessage("target NOT detected for 2 seconds.");
                logMessage("centering speed: " + centeringSpeed);
                logMessage("cached angle: " + cacheLimelightAngle);
                return true;
            }
        } else {
            lastDetectionTime = System.currentTimeMillis();
        }

        if (isHorizontalTargetOffsetAngleErrorReached) {
            logMessage("target REACHED: " + Math.abs(limelightSubsystem.getHorizontalTargetOffsetAngle()));
            return true;
        }

        return false;
    }
}
