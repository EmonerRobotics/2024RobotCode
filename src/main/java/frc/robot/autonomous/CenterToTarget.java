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

    private boolean isCurrentAngleBiggerThanCache(double currentLimelightAngle) {
        return currentLimelightAngle > cacheLimelightAngle;
    }

    private boolean isTargetDetected() {
        return limelightSubsystem.isTargetDetected();
    }

    private double calculateCenteringSpeedWithPid(double angle) {
        return pidController.calculate(
                angle
        );
    }

    private void setLimelightCacheWithNewAngle(double currentLimelightAngle) {
        cacheLimelightAngle = currentLimelightAngle;
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
                } else {
                    return slowDownCenteringSpeed(SPEED_DIVIDER);
                }
            case RIGHT:
                if (centeringSpeed < MINIMUM_SPEED_THRESHOLD) {
                    return MINIMUM_SPEED;
                } else {
                    return slowDownCenteringSpeed(SPEED_DIVIDER);
                }
            default:
                return 0;
        }
    }

    private void setCenteringSpeed(double centeringSpeed) {
        this.centeringSpeed = centeringSpeed;
    }

    private void updateCenteringSpeedForAnamolies(
            double currentLimelightAngle
    ) {
        switch (getStartingPosition()) {
            case LEFT:
                if (isCurrentAngleBiggerThanCache(currentLimelightAngle)) {
                    double newSpeed = calculateCenteringSpeedWithPid(cacheLimelightAngle);
                    setCenteringSpeed(newSpeed);
                } else {
                    double newSpeed = calculateCenteringSpeedWithPid(currentLimelightAngle);
                    setCenteringSpeed(newSpeed);
                    setLimelightCacheWithNewAngle(currentLimelightAngle);
                }
                break;
            case RIGHT:
                if (!isCurrentAngleBiggerThanCache(currentLimelightAngle)) {
                    double newSpeed = calculateCenteringSpeedWithPid(cacheLimelightAngle);
                    setCenteringSpeed(newSpeed);
                } else {
                    double newSpeed = calculateCenteringSpeedWithPid(currentLimelightAngle);
                    setCenteringSpeed(newSpeed);
                    setLimelightCacheWithNewAngle(currentLimelightAngle);
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
            updateCenteringSpeedForAnamolies(
                    getCurrentLimelightAngle()
            );

            setCenteringSpeed(determineCenteringSpeedLowLimit());

            driveSubsystem.drive(
                    0,
                    0,
                    centeringSpeed,
                    true,
                    false
            );

        } else {
            logMessage("NO TARGET: " + cacheLimelightAngle);

            driveSubsystem.drive(
                    0,
                    0,
                    centeringSpeed,
                    true,
                    false
            );

        }

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

    @Override
    public boolean isFinished() {
        boolean isHorizontalTargetOffsetAngleErrorReached =
                Math.abs(limelightSubsystem.getHorizontalTargetOffsetAngle()) < HORIZONTAL_MAX_ERROR_ANGLE;

        if(!isTargetDetected()){
            logMessage("CENTER END: target NOT detected");
           // return true;
        }
        if (isHorizontalTargetOffsetAngleErrorReached) {
            logMessage("CENTER END: target REACHED: " + Math.abs(limelightSubsystem.getHorizontalTargetOffsetAngle()));
            return true;
        }

        return false;
    }
}
