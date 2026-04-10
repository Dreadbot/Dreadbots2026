package frc.robot.subsystems.slapdown;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public interface SlapdownIO {
    
    @AutoLog
    public static class SlapdownIOInputs {
        //The Volts
        public double pivotAppliedVolts = 0.0;
        //public double intakeAppliedVolts = 0.0;
        
        //RPM
        public double intakeRPM = 0.0;
        public double absolutePosition = 0.0;
    }


    // functions
    public default void updateInputs(SlapdownIOInputs inputs) {};

    public default void runPivotVoltage(double volts) {}

    public default void runIntakeVoltage(double volts) {}

    public default void setIdleMode(IdleMode pivotIdleMode, IdleMode intakeIdleMode) {};

    public default void stopMotors() {};

    public default void runIntakeRPM(double rpm) {};
}