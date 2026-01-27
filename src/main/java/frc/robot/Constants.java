package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        REAL,

        SIM,

        REPLAY
    }
    public static class IndexerConstants {
        // public static final double INTAKE_VOLTAGE = 1.5;
        // public static final double OUTAKE_VOLTAGE = -5.0;
        public static final int MOTOR_ID = 1;
    }
    public static class FlywheelConstants {
        public static final double SHOOT_VOLTAGE = 3.0;
        public static final int MOTOR_ID_1 = 13;
        public static final int MOTOR_ID_2 = 14;
    }
}
