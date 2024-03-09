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
import frc.robot.RobotContainer;
import frc.robot.commands.AutoArm;
import frc.robot.commands.FireCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.ShooterSenderCommand;
import frc.robot.commands.SlowArmDown;
import frc.robot.commands.AutoArm.PositionControl;
import frc.robot.commands.SlowArmDown.PositionController;


public class FRCPathPlanner {
    public final static SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

    public static void SetPathPlannerSettings() {
        setDashboard();
        CommandNameEntry();
    }

    public static void setDashboard() {
        SmartDashboard.putData("Auto Mod", autoChooser);
        SmartDashboard.putBoolean("is AutoBuilder configure?", AutoBuilder.isConfigured());
        SmartDashboard.putBoolean(" is pathfinding configure?", AutoBuilder.isPathfindingConfigured());
    }

    public static void CommandNameEntry() {
        NamedCommands.registerCommand("intake", new IntakeCommand(RobotContainer.getIntakeSubsystem(), RobotContainer.getMZ80(), true));
        NamedCommands.registerCommand("fire", new FireCommand());
        NamedCommands.registerCommand("shooter", new ShooterCommand());
        NamedCommands.registerCommand("slowArm", new SlowArmDown(RobotContainer.getArmSubsystem(), PositionController.ShouldBe));
        NamedCommands.registerCommand("sender", new ShooterSenderCommand(RobotContainer.getIntakeSubsystem(), RobotContainer.getMZ80(), true));
        NamedCommands.registerCommand("autoArm", new AutoArm(RobotContainer.getArmSubsystem(), RobotContainer.getLimelightSubsystem(), PositionControl.auto));
        NamedCommands.registerCommand("zeroArm", new SlowArmDown(RobotContainer.getArmSubsystem(), PositionController.Zero));
    }

}