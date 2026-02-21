package frc.robot.subsystems.hookprep;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hookprep extends SubsystemBase {
    // IO interface for hardware interaction
    private HookprepIO io;

    // Data class for inputs
    private HookprepIO.HookprepIOInputs inputs = new HookprepIO.HookprepIOInputs();

    // Constructor to initialize the subsystem with the appropriate IO implementation
    public Hookprep(HookprepIO io) {
        // Initialize the IO implementation
        this.io = io;
    }

    // Periodic method to update inputs
    @Override
    public void periodic() {
        // Update inputs from the hardware
        io.updateInputs(inputs);
    }

    // Method to create a command that runs the pulse based on joystick position
    public Command readyHook(double joystickPosition) {
        // Calculate the desired position based on the joystick input
        double desiredPosition = joystickPosition + 1.0;

        // Ensure the desired position is within the valid range of 0.0 to 1.0
        int pulseWidth = (int)(desiredPosition * 2000) + 500;

        // Log the calculated pulse width for debugging
        System.out.println("readyhook pulseWidth=" + pulseWidth);

        // Return a command that runs the pulse for a certain duration and then resets it
        return Commands.startEnd(
            () -> {
                io.runPulse(pulseWidth);
            },
            () -> {
                io.runPulse(1500);
            }
        );
    }
}
