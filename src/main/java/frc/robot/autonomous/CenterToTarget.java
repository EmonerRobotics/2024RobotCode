package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.modules.internal.drivetrain.DriveSubsystem;
import frc.robot.modules.external.limelight.LimelightSubsystem;

import static frc.robot.core.utils.LoggingUtils.logMessage;

public class CenterToTarget extends Command {

    public static CenterToTarget instance = null;

    public static final double HORIZONTAL_MAX_ERROR_ANGLE = 0.7;
    private final LimelightSubsystem limelightSubsystem = LimelightSubsystem.getInstance();
    private final PIDController pidController;
    private final DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();
    private double centeringSpeed;
    private boolean isTargetDetected;
    private double cacheLimelightAngle;

    public static CenterToTarget getInstance() {
        if (instance == null) {
            instance = new CenterToTarget();
        }
        return instance;
    }

    public CenterToTarget() {
        pidController = new PIDController(
                0.04,
                0.0,
                0.0
        );
        //  pidController.setIZone(0.01);
        pidController.setSetpoint(0);

        addRequirements(limelightSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("CENTER BASLADI");
        cacheLimelightAngle = limelightSubsystem.getEstimatedDistance();
    }

    @Override
    public void execute() {
        double currentLimelightAngle = limelightSubsystem.getHorizontalTargetOffsetAngle();
        isTargetDetected = limelightSubsystem.isTargetDetected();

        if (isTargetDetected) {
            if(currentLimelightAngle > cacheLimelightAngle){
                centeringSpeed = pidController.calculate(
                        cacheLimelightAngle
                );
                logMessage("cached: " + centeringSpeed);
                logMessage("norm: " + pidController.calculate(
                        currentLimelightAngle
                ));
            }
            else {
                centeringSpeed = pidController.calculate(
                        currentLimelightAngle
                );
                cacheLimelightAngle = currentLimelightAngle;
                logMessage("standard: " + centeringSpeed);
            }
        } else {
            logMessage("Speed cached:" + centeringSpeed);
            centeringSpeed = pidController.calculate(
                    cacheLimelightAngle
            );

        }


        //double actualSpeed = centeringSpeed / (1 + (currentLimelightAngle / 90));
       // double actualSpeed = centeringSpeed / (1 + (currentLimelightDistance / 70));


        if (limelightSubsystem.getEstimatedDistance() > 222) {
      //      centeringSpeed = centeringSpeed / 6;
        } else {
     //       centeringSpeed = centeringSpeed / 3;
        }

        //26.17

        logMessage("Current Centering Speed:" + centeringSpeed);
        logMessage("Current Limelight Angle:" + currentLimelightAngle);
        logMessage("*********************************************");

        /*
        if(centeringSpeed / 3 < 0.03){
            centeringSpeed = 0.03;
        }else {
            centeringSpeed = centeringSpeed / 3;
        }
        */

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

        if (isHorizontalTargetOffsetAngleErrorReached && isTargetDetected) {
            System.out.println("CENTER end");
            return true;
        }

        return false;
    }
}
