package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoArm.PositionControl;
import frc.robot.commands.CenterToTarget.CenterChecker;
import frc.robot.commands.SlowArmDown.PositionController;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.MZ80;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class FireCommand extends SequentialCommandGroup {
    public FireCommand() {
        addCommands(
                new ParallelCommandGroup(
                        new CenterToTarget(CenterChecker.CENTER),
                        new AutoArm(PositionControl.ShouldBe)
                ),
                ShooterSenderCommand.getInstance(),
                new SlowArmDown(PositionController.Zero)
        );

    }
}
