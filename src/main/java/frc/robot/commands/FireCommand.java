package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoArm.PositionControl;
import frc.robot.commands.CenterToTarget.CenterChecker;
import frc.robot.commands.SlowArmDown.PositionController;

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
