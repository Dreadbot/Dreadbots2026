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

    public static class IndexerConstants {
        public static final double INTAKE_VOLTAGE = 1.5;
        public static final double OUTTAKE_VOLTAGE = -1.5;
        public static final double KICKER_INTAKE_VOLTAGE = 1.5;
        public static final double KICKER_OUTTAKE_VOLTAGE = -1.5;
        public static final int MOTOR_ID = 1;
        public static final int MOTOR_ID_KICKER = 2;
    }

    public static class FlywheelConstants {
        public static final double SHOOT_VOLTAGE = 1.0;
        public static final int MOTOR_ID_1 = 1;
        public static final int MOTOR_ID_2 = 14;
    }

    public static class HoodConstants {
        public static final int MOTOR_ID = 1;
        public static final int LOWER_LIMIT_SWITCH_ID = 1;
        // Conversion value for the motor's number of total rotations to the hood's angle in radians
        // This number should be calibrated based on the gear ratio between the motor and the hood
        public static final double ROTATIONS_TO_RADIANS = 2 * Math.PI;
        public static final double MAX_ANGLE = 2.0; // Radians
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