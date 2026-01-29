package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs {
        //Volts
        public double pivotAppliedVolts = 0.0;

        //Current
        public double pivotCurrentAmps = 0.0;

        //Rotating By Degrees
        public double pivotRotationDegrees = 0.0;
        
        //RPM
        public double piviotRPM = 0.0;
        public double absolutePosition = 0.0;

        //Temp
        public double pivotTemperature = 0.0;
    }

    public default void updateInputs(TurretIOInputs inputs) {};

    public default void runPivotVoltage(double volts) {}
}
