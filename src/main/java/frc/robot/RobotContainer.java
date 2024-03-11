// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autonomous.FireCommand;
import frc.robot.autonomous.PathPlanner;
import frc.robot.core.Constants;
import frc.robot.core.Robot;
import frc.robot.core.enums.PositionType;
import frc.robot.drivetrain.DriveWithJoysticks;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.modules.internal.arm.ArmSubsystem;
import frc.robot.modules.internal.arm.commands.ArmCommand;
import frc.robot.modules.internal.arm.commands.ArmLockCommand;
import frc.robot.modules.internal.arm.commands.ManuelArmCommand;
import frc.robot.modules.internal.intake.IntakeType;
import frc.robot.modules.internal.intake.commands.IntakeCommand;
import frc.robot.modules.internal.shooter.commands.ShooterCommand;
import frc.robot.modules.internal.shooter.commands.ShooterSenderCommand;
import frc.robot.pose_estimation.PoseEstimation;

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
                IntakeCommand.getInstance(IntakeType.REVERSE)
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
                IntakeCommand.getInstance(IntakeType.STANDARD)
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
                ArmLockCommand.getInstance()
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


