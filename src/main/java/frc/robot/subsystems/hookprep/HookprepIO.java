package frc.robot.subsystems.hookprep;

import org.littletonrobotics.junction.AutoLog;

public interface HookprepIO {
    // Data class for inputs
    @AutoLog
    public static class HookprepIOInputs {
        // Add any inputs you need here, for example:
        // public double someSensorValue = 0.0;
    }
    
    // Method to update inputs
    public default void updateInputs(HookprepIOInputs inputs) {}
    
    // Method to run pulse
    public default void runPulse(double pulseWidth) {};
}
