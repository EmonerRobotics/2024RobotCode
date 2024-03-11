package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;

import java.util.Optional;

public class AllianceUtils {

    static Optional<Alliance> ally = DriverStation.getAlliance();

    public static boolean isBlue() {
        if (ally.isPresent()) {
            return ally.get().equals(Alliance.Red);
        } else {
            return true;
        }
    }

    public static Rotation2d getFieldOrientationZero() {
        return Rotation2d.fromRadians(isBlue() ? 0 : Math.PI);
    }
}
