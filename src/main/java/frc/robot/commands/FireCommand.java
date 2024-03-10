package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CenterToTarget.CenterChecker;
import frc.robot.commands.arm.ArmCommand;
import frc.robot.commands.shooter.ShooterSenderCommand;
import frc.robot.enums.PositionType;

public class FireCommand extends SequentialCommandGroup {
    public FireCommand() {
        addCommands(
                new ParallelCommandGroup(
                        new CenterToTarget(CenterChecker.CENTER),
                        ArmCommand.getInstance(PositionType.TARGET)
                ),
                ShooterSenderCommand.getInstance(),
                ArmCommand.getInstance(PositionType.GROUND)
        );

    }
}
