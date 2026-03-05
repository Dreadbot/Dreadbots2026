package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    // Interface for the Hood subsystem's input and output, allowing for different implementations (e.g., real hardware, simulation, testing) while keeping the main Hood class decoupled from specific hardware details.
    @AutoLog
    public static class HoodIOInputs {
        public double rotations = 0.0;
        public double appliedVolts = 0.0;
        public boolean lowerSwitch = false;
    }
    // Default methods for updating inputs and controlling the hood, which can be overridden by specific implementations to interact with hardware or simulation as needed.
    default void updateInputs(HoodIOInputs inputs) {}
    // Default method to set the motor voltage, which can be overridden to control the actual motor in a real implementation or to simulate motor behavior in a testing environment.
    default void setVoltage(double volts) {}
    // Default method to set the position of the hood, which can be overridden to implement position control in a real implementation or to simulate position changes in a testing environment. This method is called when the lower limit switch is triggered to reset the hood's position to 0.
    default void setPosition(double position) {}
}