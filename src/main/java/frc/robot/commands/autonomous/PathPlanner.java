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
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.arm.ArmCommand;
import frc.robot.commands.shooter.ShooterCommand;
import frc.robot.commands.shooter.ShooterSenderCommand;
import frc.robot.enums.PositionType;


public class PathPlanner {
    public final static SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

    public static void setPathPlannerSettings() {
      //  setDashboard();
     //   commandNameEntry();
    }

    public static void setDashboard() {
        SmartDashboard.putData("Auto Mod", autoChooser);
        SmartDashboard.putBoolean("is AutoBuilder configure?", AutoBuilder.isConfigured());
        SmartDashboard.putBoolean("is pathfinding configure?", AutoBuilder.isPathfindingConfigured());
    }

    public static void commandNameEntry() {
        NamedCommands.registerCommand("intake", IntakeCommand.getInstance());
        //NamedCommands.registerCommand("fire", new FireCommand());
        NamedCommands.registerCommand("shooter", ShooterCommand.getInstance());
       // NamedCommands.registerCommand("slowArm", ArmCommand.getInstance(PositionType.TARGET));
        NamedCommands.registerCommand("sender", ShooterSenderCommand.getInstance());
        //NamedCommands.registerCommand("autoArm", ArmCommand.getInstance(PositionType.AUTO));
        //NamedCommands.registerCommand("zeroArm", ArmCommand.getInstance(PositionType.GROUND));
    }

}