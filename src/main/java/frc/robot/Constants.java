package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        REAL,

        SIM,

        REPLAY
    }

    public static final class AutoAlignConstants {
    public static final double TRANSLATION_KP = 0.5;
    public static final double TRANSLATION_KD = 0.0;
    public static final double TRANSLATION_VELOCITY = 3.5; // Meters/Sec
    public static final double TRANSLATION_ACCELERATION = 3.5; // Meters/Sec^2
    public static final double TRANSLATION_JERK = 3.0; // Meters/Sec^3
    public static final double ROTATION_KP = 2.0;
    public static final double ROTATION_KD = 0.0;
    public static final double ROTATION_MAX_VELOCITY = 5.0;
    public static final double ROTATION_MAX_ACCELERATION = 10.0;
    public static final double LEFT_REEF_BRANCH_OFFSET = Units.inchesToMeters(11.5 / 2.0);
    public static final double RIGHT_REEF_BRANCH_OFFSET = Units.inchesToMeters(13.5 / 2.0);

  }
}
