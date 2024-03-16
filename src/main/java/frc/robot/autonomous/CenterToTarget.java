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
                0.042,
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
            logMessage(String.valueOf(centeringSpeed));

            //centering speed daha onceden bir degiskene atanmamis
            if(centeringSpeed < 0) {
                logMessage("soldan");
                //tX > tag ile aramdaki mesafeden neredeyse hicbir zaman buyuk olamaz
                if(currentLimelightAngle > cacheLimelightAngle){
                    //bazen gelen 219 gibi sayilar bu yuzden olabilir
                    //centering speed ilk kez burda hesaplaniyor ancak 56. satirda kontrol ediliyor
                    centeringSpeed = pidController.calculate(
                            cacheLimelightAngle
                    );
                }
                else {
                    centeringSpeed = pidController.calculate(
                            currentLimelightAngle
                    );
                    cacheLimelightAngle = currentLimelightAngle;

                }
            }
            else {
                logMessage("sagdan");

                logMessage("currentLimelightAngle: " + currentLimelightAngle);
                logMessage("cacheLimelightAngle: " + cacheLimelightAngle);
                logMessage("is current bigger than cache: " + String.valueOf(currentLimelightAngle > cacheLimelightAngle));
                //tx degeri tag ile aradaki mesafeden her zaman daha kucuktur
                if(currentLimelightAngle < cacheLimelightAngle){
                    logMessage("current bigger than cache: " + String.valueOf(currentLimelightAngle > cacheLimelightAngle));

                    centeringSpeed = pidController.calculate(
                            cacheLimelightAngle
                    );
                }
                else {
                    logMessage("cache bigger than current: " + String.valueOf(currentLimelightAngle > cacheLimelightAngle));

                    centeringSpeed = pidController.calculate(
                            currentLimelightAngle
                    );
                    cacheLimelightAngle = currentLimelightAngle;
                }
            }



            if(centeringSpeed < 0) {
                if(centeringSpeed > -0.045){
                    centeringSpeed = -0.05;
                }else {
                    centeringSpeed = centeringSpeed / 3.2;
                }
            }
            else {
                if(centeringSpeed < 0.045){
                    centeringSpeed = 0.05;
                }else {
                    centeringSpeed = centeringSpeed / 3.2;
                }
            }



            driveSubsystem.drive(
                    0,
                    0,
                    centeringSpeed,
                    true,
                    false
            );


        } else {
            logMessage("else situ: " + cacheLimelightAngle);

            /*
            centeringSpeed = pidController.calculate(
                    cacheLimelightAngle
            );

             */
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
