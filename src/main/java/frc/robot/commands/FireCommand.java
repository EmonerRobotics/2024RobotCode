package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmCommand;
import frc.robot.commands.shooter.ShooterSenderCommand;
import frc.robot.enums.PositionType;

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
