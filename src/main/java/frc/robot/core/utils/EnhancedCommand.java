package frc.robot.core.utils;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.modules.external.limelight.LimelightSubsystem;
import frc.robot.modules.external.ultrasonic.MZ80;
import frc.robot.modules.internal.arm.ArmLockSubsystem;
import frc.robot.modules.internal.arm.ArmSubsystem;
import frc.robot.modules.internal.intake.IntakeSubsystem;
import frc.robot.pose_estimation.PoseEstimation;

public abstract class EnhancedCommand extends Command {
    protected final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
    protected final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    protected final LimelightSubsystem limelightSubsystem = LimelightSubsystem.getInstance();
    protected final ArmLockSubsystem armLockSubsystem = ArmLockSubsystem.getInstance();

    protected final MZ80 mz80 = MZ80.getInstance();
    protected final PoseEstimation poseEstimation = PoseEstimation.getInstance();
    protected final Drivetrain drivetrain = Drivetrain.getInstance();

    @Override
    public void initialize() {
        LoggingUtils.logEvent();
        super.initialize();
    }

    @Override
    public void execute() {
        LoggingUtils.logEvent();
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        LoggingUtils.logEvent();
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        LoggingUtils.logEvent();
        return super.isFinished();
    }
}
