package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.core.enums.PositionType;
import frc.robot.modules.internal.arm.commands.ArmCommand;
import frc.robot.modules.internal.shooter.commands.ShooterSenderCommand;

public class FireCommand {
    public Command fireCommand() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new CenterToTarget(),
                        ArmCommand.forceNewInstance(PositionType.TARGET)
                ),
                ShooterSenderCommand.forceNewInstance()
                //ArmCommand.forceNewInstance(PositionType.GROUND)
        );

    }
}
