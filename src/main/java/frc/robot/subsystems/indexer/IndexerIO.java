package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    // Data class for inputs
    @AutoLog
    public static class IndexerIOInputs {
        public double RPM = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }
    
    // Method to update inputs
    public default void updateInputs(IndexerIOInputs inputs, IndexerIOInputs kickerInputs ) {}
    
    // Method for sim file because the parameters are outdated
    public default void updateInputs(IndexerIOInputs inputs) {}
    
    // Method to run voltage
    public default void runSpindexerVoltage(double volts) {};
    
    // Method to run kicker voltage
    public default void runKickerVoltage(double volts) {};
    
    // Method to change current limit
    public default void changeCurrentLimit(double current) {};

    public double indexerMotorAppliedVolts = 0.0;
    public double indexerMotorCurrentAmps = 0.0;
    
    // Gets the voltage of the motor
    public default void runIndexerMotorVoltage(double volts) {}
}