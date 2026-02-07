package frc.robot.subsystems.hookprep;

public class HookprepIOSim implements HookprepIO {
    // Simulated inputs
    private double simulatedVoltage = 0.0;
    
    @Override
    public void runPulse(double volts) {
        simulatedVoltage = volts;
    }
}
