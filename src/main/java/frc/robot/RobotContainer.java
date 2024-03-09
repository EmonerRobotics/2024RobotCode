// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.commands.AutoArm.PositionControl;
import frc.robot.commands.SlowArmDown.PositionController;
import frc.robot.commands.autonomous.FRCPathPlanner;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.AllianceUtils;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    public static final Joystick swerveJoystick = new Joystick(Constants.JoystickConstants.SwerveJoystick);
    public static final Joystick upSystemJoystick = new Joystick(Constants.JoystickConstants.UpSystem);

    public static Field2d field = new Field2d();

    public RobotContainer() {
        FRCPathPlanner.SetPathPlannerSettings();
        setupDefaults();
        setupAutoBalanceStartingPosition();
        configureUpSystemJoystickBindings();
        configureSwerveJoystickBindings();
    }

    private void configureUpSystemJoystickBindings() {
        new JoystickButton(
                upSystemJoystick,
                1
        ).whileTrue(
                ShooterSenderCommand.getInstance()
        );

        new JoystickButton(
                upSystemJoystick,
                3
        ).whileTrue(
                ReverseIntake.getInstance()
        );

        //TODO: test with real robot
        new JoystickButton(
                upSystemJoystick,
                4
        ).whileTrue(
                AutoArm.getInstance(PositionControl.Amphi)
        );

        new JoystickButton(
                upSystemJoystick,
                5
        ).whileTrue(
                IntakeCommand.getInstance()
        );

        new JoystickButton(
                upSystemJoystick,
                6
        ).whileTrue(
                ShooterCommand.getInstance()
        );

        new JoystickButton(
                upSystemJoystick,
                9
        ).whileTrue(
                ReverseArmLock.getInstance()
        );
    }

    private void configureSwerveJoystickBindings() {
        new JoystickButton(
                swerveJoystick,
                8
        ).whileTrue(
                new RunCommand(
                        Drivetrain.getInstance()::setX,
                        Drivetrain.getInstance()
                )
        );

        new JoystickButton(
                swerveJoystick,
                9
        ).onTrue(
                new InstantCommand(() -> {
                    PoseEstimation.getInstance().resetPose(
                            new Pose2d(
                                    PoseEstimation.getInstance().getEstimatedPose().getTranslation(),
                                    new Rotation2d()
                            )
                    );
                })
        );

        new JoystickButton(
                swerveJoystick,
                10
        ).whileTrue(
                ArmLockCommand.getInstance()
        );

    }

    public Command getAutonomousCommand() {
        return new ParallelCommandGroup(
                ShooterCommand.getInstance(),
                new SequentialCommandGroup(
                        new SlowArmDown(PositionController.ShouldBe),
                        new WaitCommand(0.3),
                        new ShooterSenderCommand(),
                        new SlowArmDown(PositionController.Zero)
                )
        );

    }

    private void setupDefaults() {
        Drivetrain.getInstance().setDefaultCommand(DriveWithJoysticks.getInstance(swerveJoystick));

        ArmSubsystem.getInstance().setDefaultCommand(
                new ArmCommand(() -> upSystemJoystick.getRawAxis(1))
        );
    }

    private void setupAutoBalanceStartingPosition() {
        FieldObject2d autoBalanceStartingPosition = field.getObject("Auto Balance Starting Position");
        if (autoBalanceStartingPosition.getPoses().isEmpty()) {
            autoBalanceStartingPosition.setPose(AllianceUtils.allianceToField(
                            new Pose2d(
                                    new Translation2d(
                                            0,
                                            0
                                    ),
                                    new Rotation2d()
                            )
                    )
            );
        }
    }

}


