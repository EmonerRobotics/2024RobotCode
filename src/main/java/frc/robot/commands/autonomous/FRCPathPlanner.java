//10 PARÇA OTONOM

// 10 kere 1 parça al
// speakera git 
// 10 kere 1 parça at 
package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.*;
import frc.robot.commands.AutoArm.PositionControl;
import frc.robot.commands.SlowArmDown.PositionController;
import frc.robot.subsystems.ArmSubsystem;


public class FRCPathPlanner {
    public final static SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

    public static void SetPathPlannerSettings() {
        setDashboard();
        CommandNameEntry();
    }

    public static void setDashboard() {
        SmartDashboard.putData("Auto Mod", autoChooser);
        SmartDashboard.putBoolean("is AutoBuilder configure?", AutoBuilder.isConfigured());
        SmartDashboard.putBoolean("is pathfinding configure?", AutoBuilder.isPathfindingConfigured());
    }

    public static void CommandNameEntry() {
        NamedCommands.registerCommand("intake", IntakeCommand.getInstance());
        //NamedCommands.registerCommand("fire", new FireCommand());
        NamedCommands.registerCommand("shooter", ShooterCommand.getInstance());
        NamedCommands.registerCommand("slowArm", new SlowArmDown(PositionController.ShouldBe)
        );
        NamedCommands.registerCommand("sender", ShooterSenderCommand.getInstance());
        NamedCommands.registerCommand("autoArm", AutoArm.getInstance(PositionControl.auto));
        NamedCommands.registerCommand("zeroArm", new SlowArmDown(PositionController.Zero)
        );
    }

}