package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autonomous.enums.CenteringStartPosition;
import frc.robot.modules.internal.drivetrain.DriveSubsystem;
import frc.robot.modules.external.limelight.LimelightSubsystem;

import static frc.robot.core.utils.LoggingUtils.logEvent;

public class CenterToTarget extends Command {

    public static CenterToTarget instance = null;

    public static final double HORIZONTAL_MAX_ERROR_ANGLE = 0.5;
    public static final double MINIMUM_SPEED_THRESHOLD = 0.045;
    public static final double MINIMUM_SPEED = 0.05;
    public static final double SPEED_DIVIDER = 3.2;

    private final LimelightSubsystem limelightSubsystem = LimelightSubsystem.getInstance();
    private final PIDController pidController;
    private final DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();
    private double centeringSpeed;
    private double cacheLimelightAngle;

    private boolean isCenterToTargetActive = true;

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

    public boolean getIsCenterToTargetActive(){
        return isCenterToTargetActive;
    }

    private double getCurrentLimelightAngle() {
        return limelightSubsystem.getHorizontalTargetOffsetAngle();
    }

    private void setCenteringSpeed(double centeringSpeed) {
        SmartDashboard.putNumber("CENTERING_SPEED", centeringSpeed);
//        logMessage("CENTERING_SPEED_ " + centeringSpeed);
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
                  //  logMessage("LEFT MINIMUM");
                    return -MINIMUM_SPEED;
                }
                break;
            case RIGHT:
                if (centeringSpeed < MINIMUM_SPEED_THRESHOLD) {
                  //  logMessage("RIGHT MINIMUM");
                    return MINIMUM_SPEED;
                }
                break;
        }

        return centeringSpeed;


    }

    private void updateCenteringSpeedForAnomalies(
            double currentLimelightAngle
    ) {
        double newSpeed;
        switch (getStartingPosition()) {
            case LEFT:
               // logMessage("LEFT ANOMALIES");
                if (!isAnomalyDetectedForLeft(currentLimelightAngle)) {
                    newSpeed = calculateCenteringSpeedWithPid(currentLimelightAngle);
                    setCenteringSpeed(newSpeed);
               //     logMessage("selam 1");
                    setLimelightCacheWithNewAngle(currentLimelightAngle);
                } else {
               //     logMessage("selam 2");
                    newSpeed = calculateCenteringSpeedWithPid(cacheLimelightAngle);
                    setCenteringSpeed(newSpeed);
                }
                break;
            case RIGHT:
             //   logMessage("RIGHT ANOMALIES");
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
        cacheLimelightAngle = limelightSubsystem.getHorizontalTargetOffsetAngle();
        double calcres = calculateCenteringSpeedWithPid(cacheLimelightAngle);
        setCenteringSpeed(calcres);
        logEvent();
       // logMessage("initialized:" + cacheLimelightAngle);
       // logMessage("initialized:" + calcres);
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
                isCenterToTargetActive = false;
                return true;
            }
        } else {
            lastDetectionTime = System.currentTimeMillis();
        }

        if (isHorizontalTargetOffsetAngleErrorReached && isTargetDetected()) {
            isCenterToTargetActive = false;
            return true;
        }

        return false;
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
}
