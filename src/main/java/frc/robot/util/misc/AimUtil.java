package frc.robot.util.misc;

import java.util.function.DoubleSupplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
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

    public static Translation2d getFieldShiftFromJoystick(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
            return new Translation2d(-ySupplier.getAsDouble() * 3, -xSupplier.getAsDouble() * 3);
        } else {
            return new Translation2d(ySupplier.getAsDouble() * 3, xSupplier.getAsDouble() * 3);
        }
    }

    public static Translation2d getPassTranslation(Pose2d pose2d) {
        double x;
        double y;
        boolean rightSide = pose2d.getY() < 4.02;

        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
            x = 2.0;
        } else {
            x = 14.5;
        }
        
        if (rightSide) {
            y = 2.0;
        } else {
            y = 6.04;
        }

        Translation2d passTranslation = new Translation2d(x, y);

        return passTranslation;
    }

}
