package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.modules.internal.drivetrain.DriveSubsystem;
import frc.robot.modules.external.limelight.LimelightSubsystem;

import static frc.robot.core.utils.LoggingUtils.logMessage;

import com.kauailabs.navx.frc.AHRS;

public class CenterToTarget extends Command {

  public static CenterToTarget instance = null;

  public static final double HORIZONTAL_MAX_ERROR_ANGLE = 0.7;
  private final LimelightSubsystem limelightSubsystem = LimelightSubsystem.getInstance();
  private final PIDController pidController;
  private final DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();
  private double centeringSpeed;
  private boolean isTargetDetected;
  private double cacheLimelightAngle;
  private AHRS m_gyro = new AHRS(Port.kMXP);

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
        0.0);
    pidController.setSetpoint(0);

    addRequirements(limelightSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("CENTER BASLADI");
    if (isTargetDetected) {
      cacheLimelightAngle = limelightSubsystem.getHorizontalTargetOffsetAngle();
    } else {
      logMessage("else situ: " + cacheLimelightAngle);
      /*
       * centeringSpeed = pidController.calculate(
       * cacheLimelightAngle
       * );
       * 
       */
    }

    logMessage(String.valueOf(cacheLimelightAngle));
  }

  @Override
  public void execute() {
    // Cikarma islemi degistirlebilir.
    double currentLimelightAngle = m_gyro.getYaw() - cacheLimelightAngle;
    isTargetDetected = limelightSubsystem.isTargetDetected();
    logMessage(String.valueOf(centeringSpeed));

    if (centeringSpeed < 0) {
      logMessage("soldan");
      if (currentLimelightAngle > cacheLimelightAngle) {
        centeringSpeed = pidController.calculate(
            cacheLimelightAngle);
      } else {
        centeringSpeed = pidController.calculate(
            currentLimelightAngle);
        cacheLimelightAngle = currentLimelightAngle;

      }
    } else {
      if (currentLimelightAngle < cacheLimelightAngle) {
        centeringSpeed = pidController.calculate(
            cacheLimelightAngle);
      } else {
        centeringSpeed = pidController.calculate(
            currentLimelightAngle);
        cacheLimelightAngle = currentLimelightAngle;
      }
    }

    if (centeringSpeed < 0) {
      if (centeringSpeed > -0.045) {
        centeringSpeed = -0.05;
      } else {
        centeringSpeed = centeringSpeed / 3.2;
      }
    } else {
      if (centeringSpeed < 0.045) {
        centeringSpeed = 0.05;
      } else {
        centeringSpeed = centeringSpeed / 3.2;
      }
    }

    driveSubsystem.drive(
        0,
        0,
        centeringSpeed,
        true,
        false);

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
    boolean isHorizontalTargetOffsetAngleErrorReached = Math
        .abs(limelightSubsystem.getHorizontalTargetOffsetAngle()) < HORIZONTAL_MAX_ERROR_ANGLE;

    if (isHorizontalTargetOffsetAngleErrorReached || !isTargetDetected) {
      System.out.println("CENTER end");
      return true;
    }

    return false;
  }
}
