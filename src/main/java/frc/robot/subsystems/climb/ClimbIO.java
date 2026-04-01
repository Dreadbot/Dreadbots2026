package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {

    //Autologging the Rotations Per Minute, the Applied Volts to the motor and the Current Amps of the motor
    @AutoLog
    class ClimbIOInputs {
        //Screw Rotations
        public double RPM = 0.0;
        public double absolutePosition = 0.0;

        //The Volts being applied to Screw Motor
        public double appliedVolts = 0.0;

         //The Amps Curently in Screw Motor
        public double currentAmps = 0.0;    
    }

    // functions

    //Updates inputs of Climb IO
     public default void updateInputs(ClimbIOInputs inputs) {};

    // Updates the runVoltage command to update how many volts it needs to run
     public default void runVoltage(double volts) {};

    //Changes the current limit of the volts on the motor
     public default void changeCurrentLimit(double current) {};
     
         //The Volts
        public double screwMotorAppliedVolts = 0.0;

        //Current 
        public double screwMotorCurrentAmps = 0.0;
    
        //gets the voltage of the motor
        public default void runScrewMotorVoltage(double volts) {}

        public default void stopMotors() {};
}