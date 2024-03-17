package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.core.enums.PositionType;
import frc.robot.modules.internal.arm.commands.ArmCommand;
import frc.robot.modules.internal.arm.commands.ArmCommandCallback;
import frc.robot.modules.internal.shooter.commands.ShooterSenderCommand;


public class FireCommand {
    public Command fireCommand() {
        ArmCommandCallback armCommandCallback = new ArmCommandCallback() {
            @Override
            public void shoot() {
              //  logMessage("Shooting*********************************");
                CommandScheduler.getInstance().schedule(ShooterSenderCommand.getInstance());
            }
        };

        return new ParallelCommandGroup(
                new CenterToTarget(),
                ArmCommand.forceNewInstance(
                        PositionType.TARGET,
                        armCommandCallback
                )
        );
        // ShooterSenderCommand.forceNewInstance()
        //ArmCommand.forceNewInstance(PositionType.GROUND)


    }
}
