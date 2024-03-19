package frc.robot.autonomous;


import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.core.enums.PositionType;
import frc.robot.modules.internal.arm.commands.ArmCommand;
import frc.robot.modules.internal.arm.commands.ArmCommandCallback;
import frc.robot.modules.internal.shooter.commands.ShooterSenderCommand;

import static frc.robot.core.utils.LoggingUtils.logEvent;

public class FireCommand extends ParallelCommandGroup {
    public FireCommand() {

        ArmCommandCallback armCommandCallback = new ArmCommandCallback() {
            @Override
            public void shoot() {
                logEvent();
                CommandScheduler.getInstance().schedule(ShooterSenderCommand.getInstance());
            }
        };

        addCommands(
                CenterToTarget.getInstance(),
                ArmCommand.forceNewInstance(
                        PositionType.TARGET,
                        armCommandCallback
                )

        );

    }
}