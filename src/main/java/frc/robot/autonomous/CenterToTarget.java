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
    private boolean isTargetDetected;
    private double cacheLimelightAngle;
    private double currentLimelightAngle;

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

    @Override
    public void initialize() {
        System.out.println("CENTER BASLADI");
        cacheLimelightAngle = limelightSubsystem.getHorizontalTargetOffsetAngle();
        logMessage(String.valueOf(cacheLimelightAngle));
    }

    @Override
    public void execute() {
        currentLimelightAngle = limelightSubsystem.getHorizontalTargetOffsetAngle();
        isTargetDetected = limelightSubsystem.isTargetDetected();

        if (isTargetDetected) {
            logMessage(String.valueOf(centeringSpeed));

            updateCenteringSpeedForAnamolies();

            setCenteringSpeedLowLimit();

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

    private boolean isCurrentAngleBiggerThanCache(){
        return currentLimelightAngle > cacheLimelightAngle;
    }

    private void setCenteringSpeedWithPid(
            double angle
    ) {
        centeringSpeed = pidController.calculate(
                angle
        );
    }

    private void setLimelightCacheWithNewAngle() {
        cacheLimelightAngle = currentLimelightAngle;
    }

    public CenteringStartPosition getStartingPosition() {
        if (centeringSpeed < 0) {
            return CenteringStartPosition.LEFT;
        } else {
            return CenteringStartPosition.RIGHT;
        }
    }

    private void updateCenteringSpeedForAnamolies() {
        switch (getStartingPosition()) {
            case LEFT:
                if (isCurrentAngleBiggerThanCache()) {
                    setCenteringSpeedWithPid(cacheLimelightAngle);
                } else {
                    setCenteringSpeedWithPid(currentLimelightAngle);
                    setLimelightCacheWithNewAngle();
                }
                break;
            case RIGHT:
                if (!isCurrentAngleBiggerThanCache()) {
                    setCenteringSpeedWithPid(cacheLimelightAngle);
                } else {
                    setCenteringSpeedWithPid(currentLimelightAngle);
                    setLimelightCacheWithNewAngle();
                }
                break;
        }
    }

    private void setCenteringSpeedLowLimit() {
        switch (getStartingPosition()) {
            case LEFT:
                if (centeringSpeed > -MINIMUM_SPEED_THRESHOLD) {
                    centeringSpeed = -MINIMUM_SPEED;
                } else {
                    centeringSpeed = centeringSpeed / SPEED_DIVIDER;
                }
                break;
            case RIGHT:
                if (centeringSpeed < MINIMUM_SPEED_THRESHOLD) {
                    centeringSpeed = MINIMUM_SPEED;
                } else {
                    centeringSpeed = centeringSpeed / SPEED_DIVIDER;
                }
                break;
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

        if (isHorizontalTargetOffsetAngleErrorReached || !isTargetDetected) {
            System.out.println("CENTER end");
            return true;
        }

        return false;
    }
}
