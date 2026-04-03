package frc.robot.util.misc;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.TurretConstants;

public class AimUtil {
    public static final Translation2d blueHub = new Translation2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84));
    public static final Translation2d redHub = new Translation2d(Units.inchesToMeters(651.22 - 182.11), Units.inchesToMeters(158.84));

    public static Translation2d getHubTranslation() {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
            return blueHub;
        } else {
            return redHub;
        }
    }

    public static Translation2d getTurretTranslationFromRobotPose(Pose2d robotPose) {
        Translation2d robotTranslation = robotPose.getTranslation();
        Rotation2d robotRotation = robotPose.getRotation();

        Translation2d offset = TurretConstants.TURRET_OFFSET.rotateBy(robotRotation);

        return robotTranslation.plus(offset);
    }

    public static Translation2d getFieldShiftFromJoystick(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        double y = ySupplier.getAsDouble();
        double x = xSupplier.getAsDouble();
        if (x == 0 && y == 0) {
            return Translation2d.kZero;
        }
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
            return new Translation2d(-y * 3, -x * 3);
        } else {
            return new Translation2d(y * 3, x * 3);
        }
    }

    public static Translation2d getPassTranslation(Pose2d pose2d) {
        double x;
        double y;
        boolean rightSide = pose2d.getY() < 4.02;
        double xOffset = 1.0;
        double yOffset = 2.0;

        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
            x = xOffset;
        } else {
            x = 16.5 - xOffset;
        }
        
        if (rightSide) {
            y = yOffset;
        } else {
            y = 8.04 - yOffset;
        }

        Translation2d passTranslation = new Translation2d(x, y);

        return passTranslation;
    }

}
