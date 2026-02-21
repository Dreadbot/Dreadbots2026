package frc.robot.subsystems.hookprep;

public class HookprepIOSim implements HookprepIO {
    // Simulated inputs
    private double simulatedVoltage = 0.0;
    
    // Method to run pulse in simulation
    @Override
    public void runPulse(double pulseWidthUs) {
        // Simulate running the pulse by logging the pulse width
        System.out.println("HookprepIOSim.runPulse pulseWidthUs=" + pulseWidthUs);
    }
}
