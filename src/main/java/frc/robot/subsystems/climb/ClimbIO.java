package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    @AutoLog
    class ClimbIOInputs {
        //Screw Rotations
        public double RPM = 0.0;

        //The Volts being applied to Screw Motor
        public double appliedVolts = 0.0;

         //The Amps Curently in Screw Motor
        public double currentAmps = 0.0;    
    }
     public default void updateInputs(ClimbIOInputs inputs) {};


     public default void runVoltage(double volts) {};


     public default void changeCurrentLimit(double current) {};
         //The Volts
        public double screwMotorAppliedVolts = 0.0;
    

        //Current 
        public double screwMotorCurrentAmps = 0.0;

        
        //gets the voltage of the motor
        public default void runScrewMotorVoltage(double volts) {}
}