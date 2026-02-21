package frc.robot.util.misc;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.TurretConstants;

public class AimUtil {
    public static Translation2d getHubTranslation() {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
            return new Translation2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84));
        } else {
            return new Translation2d(Units.inchesToMeters(651.22 - 182.11), Units.inchesToMeters(158.84));
        }
    }

    public static Translation2d getTurretTranslationFromRobotPose(Pose2d robotPose) {
        Translation2d robotTranslation = robotPose.getTranslation();
        Rotation2d robotRotation = robotPose.getRotation();

        Translation2d offset = TurretConstants.TURRET_OFFSET.rotateBy(robotRotation);

        return robotTranslation.plus(offset);
    }

}
