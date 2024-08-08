// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ArmLockCommand;
import frc.robot.commands.AutoArm;
import frc.robot.commands.ShooterSenderCommand;
import frc.robot.commands.SlowArmDown;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.FireCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ReverseArmLock;
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.AutoArm.PositionControl;
import frc.robot.commands.SlowArmDown.PositionController;
//import frc.robot.commands.autonomous.AutoBalance;
import frc.robot.commands.autonomous.FRCPathPlanner;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.ArmLockSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.MZ80;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.AllianceUtils;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {


  public static final ShuffleboardTab driveSettingsTab = Shuffleboard.getTab("Drive Settings");
  public static final ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
  public static final ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve");

  public static final Joystick joystick1 = new Joystick(Constants.JoystickConstants.SwerveJoystick);
  public static final Joystick joystick2 = new Joystick(Constants.JoystickConstants.UpSystem);
  
  public static final Drivetrain drivetrain = new Drivetrain();
 
  public static final PoseEstimation poseEstimation = new PoseEstimation();

  public static final SendableChooser<String> drivePresetsChooser = new SendableChooser<>();

  public static Field2d field = new Field2d();
  public static Field2d nodeSelector = new Field2d();

//  private final FieldObject2d startingPosition = field.getObject("Starting Position");
  private final FieldObject2d autoBalanceStartingPosition = field.getObject("Auto Balance Starting Position");

  private DriveWithJoysticks driveCommand = new DriveWithJoysticks(drivetrain, poseEstimation, joystick1);
  //private AutoBalance autoBalanceCommand = new AutoBalance(drivetrain);

  public static final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public static final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public static final ArmSubsystem armSubsystem = new ArmSubsystem();
  public static final ArmLockSubsystem armlocksubsystem =new ArmLockSubsystem();
  public static final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  public static final MZ80 mz80 = new MZ80();
 

 
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    FRCPathPlanner.SetPathPlannerSettings();

    drivetrain.setDefaultCommand(driveCommand);
    configureBindings();

    armSubsystem.setDefaultCommand(new ArmCommand(armSubsystem, () ->  joystick2.getRawAxis(1)));
  }

  private void configureBindings() {

//Shooter Subsystem
    new JoystickButton(joystick2, Constants.UpSystemConstants.ShooterStarterB).whileTrue(
      new ShooterCommand(shooterSubsystem, true));


//Auto Arm Command
/*
    new JoystickButton(joystick2, 2).onTrue(
      new AutoArm(armSubsystem, limelightSubsystem, PositionControl.ShouldBe)); */

    //Intake 
    new JoystickButton(joystick2, 5).whileTrue(new IntakeCommand(intakeSubsystem, mz80, true));

    //new JoystickButton(joystick1, 2).whileTrue(new )

    //Kontrolsuz Intake
    new JoystickButton(joystick2, 1).whileTrue(new ShooterSenderCommand(intakeSubsystem, mz80, true));

    //Intake Reverse
    new JoystickButton(joystick2, 3).whileTrue(new ReverseIntake(intakeSubsystem, true));

    //AMPHI PID
    new JoystickButton(joystick2,4).whileTrue(new AutoArm(armSubsystem, limelightSubsystem, PositionControl.Amphi));
    
    //Fire Command
    new JoystickButton(joystick2, 2).onTrue(new FireCommand());

    /*
    new JoystickButton(
      joystick2,
       2).onTrue(new SequentialCommandGroup(
    new AutoArm(armSubsystem, limelightSubsystem, PositionControl.auto),
    new ShooterSenderCommand(intakeSubsystem, mz80, true)));
     */
     

    //new JoystickButton(joystick2, 2).onTrue(new FireCommand());
    
    //Kanca kilit
    new JoystickButton(joystick1, 10).whileTrue(new ArmLockCommand(armlocksubsystem, true));

    //Safe Driving 45 Degrees
    //new JoystickButton(joystick1, 7).whileTrue(new AutoArm(armSubsystem, limelightSubsystem, PositionControl.Cover));
    //new JoystickButton(joystick1, 7).toggleOnTrue(new SafeDrive(armSubsystem, limelightSubsystem, Position.Cover));

    //Zero Arm
    //new JoystickButton(joystick1, 5).whileTrue(new SlowArmDown(armSubsystem, PositionController.ShouldBe));

    //Kanca kilit Reverse
    new JoystickButton(joystick2, 9).whileTrue(new ReverseArmLock(armlocksubsystem, true));

    //new JoystickButton(joystick2, 8).onTrue(new AutoArm(armSubsystem, limelightSubsystem, null))


// Pose Estimation
/*
    new JoystickButton(joystick1, 10).//6
      onTrue(new InstantCommand(driveCommand::resetFieldOrientation)); */

    new JoystickButton(joystick1, 9).//7
      onTrue(new InstantCommand(() -> poseEstimation.resetPose(
      new Pose2d(
        poseEstimation.getEstimatedPose().getTranslation(),
        new Rotation2d())))); 

// Driving 
 
    new JoystickButton(joystick1, 8).
      whileTrue(new RunCommand(
      drivetrain::setX,
      drivetrain)); 

    //new JoystickButton(joystick1, 2).//3
      //whileTrue(autoBalanceCommand);
  }
   
  public Command getAutonomousCommand() {
    //return new PathPlannerAuto("testA");
    

    return new ParallelCommandGroup(

    new ShooterCommand(shooterSubsystem, true),

    new SequentialCommandGroup(
        new SlowArmDown(armSubsystem, PositionController.ShouldBe),
        //new AutoArm(armSubsystem, limelightSubsystem, PositionControl.ShouldBe),
        new WaitCommand(0.3),        

        new ShooterSenderCommand(intakeSubsystem,mz80, true),
        new SlowArmDown(armSubsystem, PositionController.Zero),
        new PathPlannerAuto("last")
         
        //new PathPlannerAuto("Auto2.0")
        //new PathPlannerAuto("Auto3.0")

       )
    );
    

    //return new ParallelCommandGroup(
    //  new ShooterCommand(shooterSubsystem, true),
  //    new PathPlannerAuto("Test3")
 //   );

    /*

    return new ParallelCommandGroup(
      
    new ShooterCommand(shooterSubsystem, true),
        new SequentialCommandGroup(
        new WaitCommand(3),
        new ShooterSenderCommand(intakeSubsystem, mz80, true),
        new PathPlannerAuto("Test3")
        )
    ); */
    
  }
    
  /*
    return new SequentialCommandGroup(
    

    new SequentialCommandGroup(
      

      new ParallelRaceGroup(

      new ShooterCommand(shooterSubsystem, true),

      new SequentialCommandGroup(
        new SlowArmDown(armSubsystem, PositionController.ShouldBe),
        //new AutoArm(armSubsystem, limelightSubsystem, PositionControl.ShouldBe),
        new WaitCommand(0.1),        

        new ShooterSenderCommand(intakeSubsystem, mz80,true)))),
        
        //new SlowArmDown(armSubsystem, PositionController.ShouldBe));
         */
         
    

    /*
    return new SequentialCommandGroup(
      new IntakeCommand(intakeSubsystem, true)
    ); */

    /*

    return new ParallelCommandGroup(

      new ShooterCommand(shooterSubsystem, true),

    new SequentialCommandGroup(
        new SlowArmDown(armSubsystem, limelightSubsystem, PositionController.ShouldBe),
        new AutoArm(armSubsystem, limelightSubsystem, PositionControl.ShouldBe),        

        new AutoIntakeCommand(intakeSubsystem, true),
        new WaitCommand(0.3),
        new AutoIntakeCommand(intakeSubsystem, false)
       )
    );
     */
    /*
    Pose2d startingPose = startingPosition.getPose();
    PathPlannerPath path = PathPlannerPath.fromPathFile("fly");
    return new SequentialCommandGroup(
      new InstantCommand(() -> poseEstimation.resetPose(
      new Pose2d(
        poseEstimation.getEstimatedPose().getTranslation(),
        new Rotation2d()))),
    new InstantCommand(() -> poseEstimation.resetPose(startingPose)),
    //AutoBuilder.followPath(path)
    FRCPathPlanner.autoChooser.getSelected()
    //FRCPathPlanner.followPathCommand("fly")
    //new FireCommand()
    //3 AutoBuilder.followPath(path) 
    ); */
 

  
 public static ShooterSubsystem getShooterSubsystem(){
    return shooterSubsystem;
  }

  public static Drivetrain getSwerveSubsystem() {
    return drivetrain;
  }

  public static LimelightSubsystem getLimelightSubsystem(){
    return limelightSubsystem;
  }

  public static PoseEstimation getPoseEstimation(){
    return poseEstimation;
  }

  public static ArmSubsystem getArmSubsystem(){
    return armSubsystem;
  }

  public static IntakeSubsystem getIntakeSubsystem(){
    return intakeSubsystem;
  }


  public static MZ80 getMZ80(){
    return mz80;
  }

}


