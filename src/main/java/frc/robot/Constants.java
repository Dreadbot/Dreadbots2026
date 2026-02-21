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

    public static class IndexerConstants {
        public static final double SPINDEXER_VOLTAGE = 1.5;
        public static final double KICKER_VOLTAGE = 1.5;
        public static final int SPINDEXER_MOTOR_ID = 1;
        public static final int KICKER_MOTOR_ID = 2;
    }

    public static class FlywheelConstants {
        public static final double SHOOT_VOLTAGE = 1.0;
        public static final int MOTOR_ID_1 = 14;
        public static final int MOTOR_ID_2 = 13;
        public static final double RPM_TOLERANCE = 100.0;
    }

    public static class HoodConstants {
        public static final int MOTOR_ID = 18;
        public static final int LOWER_LIMIT_SWITCH_ID = 1;
        // Conversion value for the motor's number of total rotations to the hood's angle in radians
        // This number should be calibrated based on the gear ratio between the motor and the hood
        public static final double GEAR_RATIO = 5; // Probably more like 75 or 150 but this was for testing
        public static final double MOTOR_ROTATIONS_TO_HOOD_RADIANS = (2 * Math.PI) / GEAR_RATIO;
        public static final double MAX_ROTATIONS = 50; // Placeholder value
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
    public static final double TURRET_ENCODER_OFFSET_RAD = 0;//Math.PI;
    public static final double TURRET_ZERO_ROBOT_RELATIVE = Math.PI / 2.0;
    public static final double GEAR_REDUCTION = 50.0;
    public static final double MAX_VOLTAGE = 9999.0;
    public static final double TURRET_MOI = 0.0366;

    public static final Translation2d TURRET_OFFSET = new Translation2d(0.0, 0.0);//new Translation2d(-0.1518, -0.210);     // +X = -Y in CAD +Y = +X in CAD (offset from center)

    public static final double MAX_ANGLE_RAD = Units.degreesToRadians(185);   // +180
    public static final double MIN_ANGLE_RAD = -Units.degreesToRadians(185);  // -180

    public static final double TURRET_Kp = 9.0;
    public static final double TURRET_Kd = 0.2;
    public static final double TURRET_Ks = 0.4;
  }

  public static class AutoAimConstants {
    public static final double PHASE_DELAY = 0.02;
  }
  public static class ClimbConstants {
    public static final double TRANSLATION_KP = 0.5;
    public static final double TRANSLATION_KD = 0.0;
    public static final double TRANSLATION_VELOCITY = 3.5; // Meters/Sec
    public static final double TRANSLATION_ACCELERATION = 3.5; // Meters/Sec^2
    public static final double TRANSLATION_JERK = 3.0; // Meters/Sec^3
    public static final double ROTATION_KP = 2.0;
    public static final double ROTATION_KD = 0.0;
    public static final double ROTATION_MAX_VELOCITY = 5.0;
    public static final double ROTATION_MAX_ACCELERATION = 10.0;
    public static final double RAISE_VOLTAGE = 0.5;
    public static final double LOWER_VOLTAGE = -0.5;
    public static final double LEVEL_ONE_CLIMB_POSITION = -1.0; // Placeholder value, should be calibrated based on the actual climb mechanism
    public static final double HOOK_DISENGAGE_POSITION = -2.0; // Value where the hooks go in
    public static final int MOTOR_ID = 1;
    public static final int LOWER_LIMIT_SWITCH_ID = 1;
    public static final int UPPER_LIMIT_SWITCH_ID = 2;
    public static final double PIDCONTROLLER_KP = 0.013;
    public static final double PIDCONTROLLER_KD = 0.00;
    public static final double ARMFEEDFORWARD_KS = 0.00;
    public static final double ARMFEEDFORWARD_KV = 0.023;
    public static final int TRAPEZOID_CONSTRAINTS_MAX_VELOCITY = 540;
    public static final int TRAPEZOID_CONSTRAINTS_MAX_ACCELERATION = 540;
    public static final int TRAPEZOID_STATE_POSITION = 0;
    public static final int TRAPEZOID_STATE_VELOCITY = 0;
  }

  public static class SlapdownConstants {
    public static final int SLAPDOWN_DUTY_CYCLE_ENCODER = 8;
    public static final double ENCODER_OFFSET = 0;
    public static final double HOME_ANGLE_DEGREES = 0;
    public static final double INTAKE_ANGLE_DEGREES = 0;
    public static final double MAX_ANGLE_DEGREES =  131.5;
    public static final double ENCODER_FREQUENCY = 975.6;
    public static final int INTAKE_MOTOR_ID = 20;
    public static final int PIVOT_MOTOR_ID = 18;
    public static final double INTAKE_VOLTAGE = 0;
    public static final double KP = 2;
    public static final double KD = 0;
    public static final double KS = 0;
    public static final double KV = 0;
    public static final double KG = 0;
    public static final double MAX_VELOCITY = 540;
    public static final double MAX_ACCELERATION = 540;
    public static final int SLAPDOWN_GEARING = 0;
    public static final int INTAKE_GEARING = 0;
    public static final double ARM_LENGTH = Units.inchesToMeters(11.75);
    public static final double MIN_ANGLE_RAD = Units.degreesToRadians(0);
    public static final double MAX_ANGLE_RAD = Units.degreesToRadians(131.5);

    public static final double SIM_INTAKE_MOI = 0.00011264;
    public static final double SIM_PIVOT_MOI = .11;
  }

}
