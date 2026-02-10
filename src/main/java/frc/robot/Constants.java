package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.geometry.Translation2d;
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
  }

  public static class TurretConstants {
    public static final int TURRET_DUTY_CYCLE_ENCODER = 1;
    public static final int TURRET_MOTOR_ID = 1;
    public static final double TURRET_ENCODER_OFFSET_RAD = 0.0;
    public static final double TURRET_JOYSTICK_SLEW_VALUE = 3;
    public static final double TURRET_EXPECTED_ZERO = 0;
    public static final double GEAR_REDUCTION = 50.0;
    public static final double TEST_ANGLE = Units.degreesToRadians(90);
    public static final double MAX_VOLTAGE = 9999.0;
    public static final double TURRET_MOI = 0.0366;

    public static final Translation2d TURRET_OFFSET = new Translation2d(0.0, 0.0);     // +X = Towards front +Y = Towards right

    public static final double MAX_ANGLE_RAD = Math.PI;   // +180
    public static final double MIN_ANGLE_RAD = -Math.PI;  // -180

    public static final double TURRET_LOOKAHEAD = 0.02;
  }
}
