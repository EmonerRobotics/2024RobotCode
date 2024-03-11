// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.FireCommand;
import frc.robot.commands.arm.ArmCommand;
import frc.robot.commands.arm.ArmLockCommand;
import frc.robot.commands.arm.ManuelArmCommand;
import frc.robot.commands.arm.ReverseArmLock;
import frc.robot.commands.autonomous.PathPlanner;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.ReverseIntake;
import frc.robot.commands.shooter.ShooterCommand;
import frc.robot.commands.shooter.ShooterSenderCommand;
import frc.robot.enums.PositionType;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    public static final Joystick swerveJoystick = new Joystick(Constants.JoystickConstants.SwerveJoystick);
    public static final Joystick upSystemJoystick = new Joystick(Constants.JoystickConstants.UpSystem);

    public RobotContainer() {
        PathPlanner.setPathPlannerSettings();
        setupDefaults();
        configureUpSystemJoystickBindings();
        configureSwerveJoystickBindings();
    }

    private void configureUpSystemJoystickBindings() {
        new JoystickButton(
                upSystemJoystick,
                1
        ).whileTrue(
                ShooterSenderCommand.forceNewInstance()
        );

        new JoystickButton(
                upSystemJoystick,
                2
        ).onTrue(
                new FireCommand().fireCommand()
        );

        new JoystickButton(
                upSystemJoystick,
                3
        ).whileTrue(
                ReverseIntake.getInstance()
        );

        new JoystickButton(
                upSystemJoystick,
                4
        ).whileTrue(
                ArmCommand.forceNewInstance(PositionType.AMPHI)
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
                        ArmCommand.getInstance(PositionType.TARGET),
                        new WaitCommand(0.3),
                        new ShooterSenderCommand(),
                        ArmCommand.getInstance(PositionType.GROUND)
                )
        );

    }

    private void setupDefaults() {
        Drivetrain.getInstance().setDefaultCommand(DriveWithJoysticks.getInstance(swerveJoystick));

        ArmSubsystem.getInstance().setDefaultCommand(
                new ManuelArmCommand(() -> upSystemJoystick.getRawAxis(1))
        );
    }

}


