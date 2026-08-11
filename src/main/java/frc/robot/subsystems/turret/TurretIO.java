package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs {
        //Volts
        public double turretAppliedVolts = 0.0;

        //Current
        public double turretCurrentAmps = 0.0;

        //Angle
        public double turretRotationRad = 0.0;
        public double absoluteEncoder1 = 0.0;
        public double absoluteEncoder2 = 0.0;
    }

    public default void updateInputs(TurretIOInputs inputs) {};

    public default void runTurretVoltage(double volts) {};

    public default void setZero() {};
   
}
