package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

import static edu.wpi.first.units.Units.Seconds;

import java.util.concurrent.TimeUnit;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Time;

public class Constants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        REAL,

        SIM,

        REPLAY
    }

  public static class IndexerConstants {
    public static final double SPINDEXER_VOLTAGE = 12.0;
    public static final double KICKER_VOLTAGE = 8.0;
    public static final int SPINDEXER_MOTOR_ID = 11;
    public static final double KICKER_RPM = 3000.0;
    public static final double KICKER_KP = 0.002;
    public static final double KICKER_KI = 0.0;
    public static final double KICKER_KD = 0.0;
    public static final double KICKER_KS = 0.0;
    public static final double KICKER_KV = 0.00185;
    public static final int KICKER_MOTOR_ID = 12;
    public static final double DEAD_BAND = 0.5;
  }


  public static class FlywheelConstants {
      public static final int MOTOR_ID_1 = 14;
      public static final int MOTOR_ID_2 = 13;
      public static final double RPM_TOLERANCE = 50.0;
  }

  public static final class LedConstants {
      public static final int LED_COUNT = 101;
      public static final int PWM_PORT = 0;
      public static final Time BLINK_FREQUENCY = Seconds.of(0.1);
      public static final Time BREATHE_FREQUENCY = Seconds.of(0.5);
      public static final double WARN_TIME = 3.0;
      public static final double SHOOT_SIGNAL_TIME = 2.5;
  }

  public static class HoodConstants {
    public static final int MOTOR_ID = 15;
    public static final int LOWER_LIMIT_SWITCH_ID = 3;
    // Conversion value for the motor's number of total rotations to the hood's angle in radians
    // This number should be calibrated based on the gear ratio between the motor and the hood
    public static final double HOOD_KP = 0.85;
    public static final double MAX_VOLTAGE = 2.0;
    public static final double MAX_ROTATIONS = 10.8;
    public static final double HOOD_KS = 0.50;
    public static final double HOOD_KI = 0.0;
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
    public static final int TURRET_DUTY_CYCLE_ENCODER = 0;
    public static final int TURRET_MOTOR_ID = 16;
    public static final double TURRET_ENCODER_OFFSET_RAD = 0.24;//Math.PI;
    public static final double TURRET_ZERO_ROBOT_RELATIVE = 0.76; //Math.PI / 2.0;
    public static final double GEAR_REDUCTION = 150.0;
    public static final double MAX_VOLTAGE = 12.0;
    public static final double TURRET_MOI = 0.0366;
    public static final double GOAL_TOLERANCE = Units.degreesToRadians(90);


    public static final Translation2d TURRET_OFFSET = new Translation2d(-0.1518, -0.210);     // +X = -Y in CAD +Y = +X in CAD (offset from center)

    public static final double MAX_ANGLE_RAD = Units.degreesToRadians(181);
    public static final double MIN_ANGLE_RAD = -Units.degreesToRadians(181);

    public static final double TURRET_Kp = 13.0;
    public static final double TURRET_Kd = 0.0;
    public static final double TURRET_Ks = 0.2465;
    public static final double MAX_ACCELERATION = 30;
    public static final double MAX_VELOCITY = 10;
  }

  public static class AutoAimConstants {
    public static final double PHASE_DELAY = 0.02;
    //public static boolean PREPSHOT_OVERRIDE = false;
    public static final double HOOD_LOWER_TIME = 1.0;
  }

  public static class ClimbConstants {
    public static final double RAISE_VOLTAGE = 6;
    public static final double LOWER_VOLTAGE = -6;
    public static final double LEVEL_ONE_CLIMB_POSITION = 109.0; // Placeholder value, should be calibrated based on the actual climb mechanism
    public static final double STOW_POSITION = 0.0;
    public static final double CLIMB_POSITION = 50; // Value where the hooks go in
    public static final int MOTOR_ID = 17;
    public static final int LOWER_LIMIT_SWITCH_ID = 7;
    public static final int UPPER_LIMIT_SWITCH_ID = 6;
    public static final double PIDCONTROLLER_KP = 0.3;
    public static final double PIDCONTROLLER_KI = 0.1;
    public static final double PIDCONTROLLER_KD = 0.00;
    public static final double ARMFEEDFORWARD_KS = 2.0;
    public static final double ARMFEEDFORWARD_KV = 0.0;
    public static final int TRAPEZOID_CONSTRAINTS_MAX_VELOCITY = 540;
    public static final int TRAPEZOID_CONSTRAINTS_MAX_ACCELERATION = 540;
    public static final int TRAPEZOID_STATE_POSITION = 0;
    public static final int TRAPEZOID_STATE_VELOCITY = 0;
  }

  public static class SlapdownConstants {
    public static final int SLAPDOWN_DUTY_CYCLE_ENCODER = 5;
    public static final double ENCODER_OFFSET = 60;
    public static final double HOME_ANGLE_DEGREES = 0.0;
    public static final double INTAKE_ANGLE_DEGREES = 129.9;
    public static final double MAX_ANGLE_DEGREES =  130;
    public static final double ENCODER_FREQUENCY = 975.6;
    public static final int INTAKE_MOTOR_ID = 10;
    public static final int PIVOT_MOTOR_ID = 9;
    public static final double INTAKE_VOLTAGE = 12;
    public static final double KP = 0.05;
    public static final double KI = 0.01;
    public static final double KD = 0;
    public static final double KS = 1.5;
    public static final double KV = 0.0;
    public static final double KG = 0.0;
    public static final double MAX_VELOCITY = 540;
    public static final double MAX_ACCELERATION = 540;
    public static final int SLAPDOWN_GEARING = 27;
    public static final double INTAKE_GEARING = 1.4;
    public static final double ARM_LENGTH = Units.inchesToMeters(11.75);
    public static final double MIN_ANGLE_RAD = Units.degreesToRadians(-20);
    public static final double MAX_ANGLE_RAD = Units.degreesToRadians(131.5);

    public static final double SIM_INTAKE_MOI = 0.00011264;
    public static final double SIM_PIVOT_MOI = .11;
  }

}
