package frc.robot.subsystems.indexer;


import org.littletonrobotics.junction.AutoLog;


public interface IndexerIO {


    @AutoLog
    public static class IndexerIOInputs {
        public double RPM = 0.0;


        public double appliedVolts = 0.0;


        public double currentAmps = 0.0;
    }
    public default void updateInputs(IndexerIOInputs inputs) {}


    public default void runVoltage(double volts) {};


    public default void changeCurrentLimit(double current) {};


}
