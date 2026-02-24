package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs {
        //Volts
        public double turretAppliedVolts = 0.0;

        //Current
        public double turretCurrentAmps = 0.0;

        //Rotating By Degrees
        public double turretRotationRad = 0.0;
        
        public double absolutePosition = 0.0;
    }

    public default void updateInputs(TurretIOInputs inputs) {};

    public default void runTurretVoltage(double volts) {}
}
