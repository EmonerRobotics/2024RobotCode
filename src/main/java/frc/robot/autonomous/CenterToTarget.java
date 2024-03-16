package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autonomous.enums.CenteringStartPosition;
import frc.robot.modules.internal.drivetrain.DriveSubsystem;
import frc.robot.modules.external.limelight.LimelightSubsystem;

import static frc.robot.core.utils.LoggingUtils.logMessage;

public class CenterToTarget extends Command {

    public static CenterToTarget instance = null;

    public static final double HORIZONTAL_MAX_ERROR_ANGLE = 0.7;
    public static final double MINIMUM_SPEED_THRESHOLD = 0.045;
    public static final double MINIMUM_SPEED = 0.05;
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
        logMessage("CACHE set");
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
                  //  logMessage("set minimum speed");
                    return MINIMUM_SPEED;
                } else {
                   // logMessage("divide speed");
                    return slowDownCenteringSpeed(SPEED_DIVIDER);
                }
            default:
                return 0;
        }
    }

    private void setCenteringSpeed(double centeringSpeed) {
        logMessage("set centering speed" + String.valueOf(centeringSpeed));
        this.centeringSpeed = centeringSpeed;
    }

    /*
    private void updateCenteringSpeedForAnamolies(
            double currentLimelightAngle
    ) {
        if (isCurrentAngleBiggerThanCache(currentLimelightAngle)) {
            updateSpeedForLeftPosition();
        } else {
            updateSpeedForRightPosition(currentLimelightAngle);
        }
    }

     */

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

    private void updateSpeedForLeftPosition() {
        logMessage("LEFT");
        double newSpeed = calculateCenteringSpeedWithPid(cacheLimelightAngle);
        setCenteringSpeed(newSpeed);
    }

    private void updateSpeedForRightPosition(double currentLimelightAngle) {
        logMessage("RIGHT");
        double newSpeed = calculateCenteringSpeedWithPid(currentLimelightAngle);
        setCenteringSpeed(newSpeed);
        setLimelightCacheWithNewAngle(currentLimelightAngle);
    }


    @Override
    public void initialize() {
        System.out.println("CENTER BASLADI");
        cacheLimelightAngle = limelightSubsystem.getHorizontalTargetOffsetAngle();
        logMessage(String.valueOf(cacheLimelightAngle));
    }

    @Override
    public void execute() {
        if (isTargetDetected()) {
            logMessage("Centering Speed: " + String.valueOf(centeringSpeed));

            updateCenteringSpeedForAnamolies(
                    getCurrentLimelightAngle()
            );

            setCenteringSpeed(determineCenteringSpeedLowLimit());
            logMessage("Centering Speed Low Limit: " + String.valueOf(determineCenteringSpeedLowLimit()));

            driveSubsystem.drive(
                    0,
                    0,
                    centeringSpeed,
                    true,
                    false
            );

        } else {
            logMessage("NO TARGET: " + cacheLimelightAngle);
        }

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
        boolean isHorizontalTargetOffsetAngleErrorReached =
                Math.abs(limelightSubsystem.getHorizontalTargetOffsetAngle()) < HORIZONTAL_MAX_ERROR_ANGLE;

        if (isHorizontalTargetOffsetAngleErrorReached || !isTargetDetected()) {
            logMessage("CENTER end");
            return true;
        }

        return false;
    }
}
