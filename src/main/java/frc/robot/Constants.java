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
        public static final double OUTAKE_VOLTAGE = -5.0;
        public static final int MOTOR_ID = 1;
    }

    public static class SlapdownConstants {
        public static final double INTAKE_VOLTAGE = -5.0;
        public static final double OUTAKE_VOLTAGE = 5.0;
        public static final int SLAPDOWN_DUTY_CYCLE_ENCODER = 8;
        public static final double ENCODER_OFFSET = 108.125;
        public static final double HOME_ANGLE_DEGREES = 0;
        public static final double OUTTAKE_ANGLE_DEGREES = 3;
        public static final double INTAKE_ANGLE_DEGREES = 60.0;
        public static final double HOLD_ANGLE_DEGREES = 14.0;
        public static final double MAX_ANGLE_DEGREES = 80.0;
        public static final double ENCODER_FREQUENCY = 975.6;
        public static final int INTAKE_MOTOR_ID = 20;
        public static final int PIVOT_MOTOR_ID = 18;

        public static final double SIM_INTAKE_MOI = 0.00011264;
        public static final double SIM_PIVOT_MOI = 0.15180934;

  }


    public static class FlywheelConstants {
        public static final double SHOOT_VOLTAGE = 3.0;
        public static final int MOTOR_ID_1 = 13;
        public static final int MOTOR_ID_2 = 14;
    }

    public static class ClimbConstants {
        public static final double INTAKE_VOLTAGE = 1.5;
        public static final double OUTAKE_VOLTAGE = -1.5;
        public static final int MOTOR_ID = 1;
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