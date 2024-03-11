package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.AllianceUtils;
import java.util.Set;

public class DriveWithJoysticks extends Command {
    private static DriveWithJoysticks instance = null;

    private final Drivetrain drivetrain = Drivetrain.getInstance();
    private final PoseEstimation poseEstimation = PoseEstimation.getInstance();
    private final Rotation2d fieldOrientationZeroOffset = new Rotation2d();
    private Joystick translation = null;

    public DriveWithJoysticks() {
        Drivetrain.getInstance().resetEncoders();
    }

    public static DriveWithJoysticks getInstance(
            Joystick translation
    ) {
        if (instance == null) {
            instance = new DriveWithJoysticks();
        }
        instance.setJoystickTranslation(translation);
        return instance;
    }

    public void setJoystickTranslation(Joystick translation) {
        this.translation = translation;
    }

    @Override
    public void execute() {
        // Negative because joysticks are inverted
        double ty = MathUtil.applyDeadband(
                -translation.getRawAxis(0),
                0.1
        );
        double tx = MathUtil.applyDeadband(
                -translation.getRawAxis(1),
                0.1
        );
        double r = MathUtil.applyDeadband(
                translation.getRawAxis(2),
                0.1
        );

        double vx = tx * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
        double vy = ty * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
        double omega = r * DriveConstants.MAX_ANGULAR_SPEED;
        ChassisSpeeds fieldRelSpeeds = new ChassisSpeeds(vx, vy, omega);
        ChassisSpeeds robotRelSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                fieldRelSpeeds,
                poseEstimation.getEstimatedPose().getRotation()
                        .minus(AllianceUtils.getFieldOrientationZero().plus(fieldOrientationZeroOffset))
        );

        drivetrain.drive(robotRelSpeeds);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(drivetrain);
    }
}
