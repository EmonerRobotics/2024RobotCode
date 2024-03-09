// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ArmLockCommand;
import frc.robot.commands.AutoArm;
import frc.robot.commands.ShooterSenderCommand;
import frc.robot.commands.SlowArmDown;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ReverseArmLock;
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.AutoArm.PositionControl;
import frc.robot.commands.SlowArmDown.PositionController;
import frc.robot.commands.autonomous.FRCPathPlanner;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.ArmLockSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.MZ80;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.AllianceUtils;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    public static final Joystick joystick1 = new Joystick(Constants.JoystickConstants.SwerveJoystick);
    public static final Joystick joystick2 = new Joystick(Constants.JoystickConstants.UpSystem);

    public static final Drivetrain drivetrain = new Drivetrain();

    public static final SendableChooser<String> drivePresetsChooser = new SendableChooser<>();

    public static Field2d field = new Field2d();
    public static Field2d nodeSelector = new Field2d();

    //  private final FieldObject2d startingPosition = field.getObject("Starting Position");
    private final FieldObject2d autoBalanceStartingPosition = field.getObject("Auto Balance Starting Position");

    private DriveWithJoysticks driveCommand = new DriveWithJoysticks(drivetrain, PoseEstimation.getInstance(), joystick1);
    //private AutoBalance autoBalanceCommand = new AutoBalance(drivetrain);
    public static final ArmLockSubsystem armlocksubsystem = new ArmLockSubsystem();
    public static final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        FRCPathPlanner.SetPathPlannerSettings();

        drivetrain.setDefaultCommand(driveCommand);

        if (autoBalanceStartingPosition.getPoses().isEmpty()) {
            autoBalanceStartingPosition.setPose(AllianceUtils.allianceToField(new Pose2d(new Translation2d(0, 0), new Rotation2d())));
        }
        configureBindings();

        ArmSubsystem.getInstance().setDefaultCommand(new ArmCommand(ArmSubsystem.getInstance(), () -> joystick2.getRawAxis(1)));
    }

    private void configureBindings() {
        new JoystickButton(
                joystick2,
                Constants.UpSystemConstants.ShooterStarterB
        ).whileTrue(
                ShooterCommand.getInstance()
        );

        new JoystickButton(
                joystick2,
                5).whileTrue(
                IntakeCommand.getInstance()
        );

        new JoystickButton(
                joystick2,
                1).whileTrue(
                ShooterSenderCommand.getInstance()
        );

        new JoystickButton(
                joystick2,
                3).whileTrue(
                ReverseIntake.getInstance()
        );

        new JoystickButton(joystick2, 4).whileTrue(
                AutoArm.getInstance()
        );

        new JoystickButton(joystick1, 10).whileTrue(
                new ArmLockCommand(armlocksubsystem, true
                ));

        new JoystickButton(joystick2, 9).whileTrue(new ReverseArmLock(armlocksubsystem, true));

        new JoystickButton(joystick1, 9).//7
                onTrue(new InstantCommand(() -> PoseEstimation.getInstance().resetPose(
                new Pose2d(
                        PoseEstimation.getInstance().getEstimatedPose().getTranslation(),
                        new Rotation2d()))));

        new JoystickButton(joystick1, 8).
                whileTrue(new RunCommand(
                        drivetrain::setX,
                        drivetrain));

    }

    public Command getAutonomousCommand() {
        return new ParallelCommandGroup(
                ShooterCommand.getInstance(),
                new SequentialCommandGroup(
                        new SlowArmDown(ArmSubsystem.getInstance(), PositionController.ShouldBe),
                        new WaitCommand(0.3),
                        new ShooterSenderCommand(IntakeSubsystem.getInstance(), MZ80.getInstance(), true),
                        new SlowArmDown(ArmSubsystem.getInstance(), PositionController.Zero)
                )
        );


    }

    public static Drivetrain getSwerveSubsystem() {
        return drivetrain;
    }

    public static LimelightSubsystem getLimelightSubsystem() {
        return limelightSubsystem;
    }

    public static ArmSubsystem getArmSubsystem() {
        return ArmSubsystem.getInstance();
    }
}


