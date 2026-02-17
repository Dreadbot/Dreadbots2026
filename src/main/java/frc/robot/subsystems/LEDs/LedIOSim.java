package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import frc.robot.Constants.LedConstants;

public class LedIOSim implements LedIO {
    private final AddressableLEDBuffer buffer;
    private LEDPattern currentPattern;
    
    public LedIOSim() {
        buffer = new AddressableLEDBuffer(LedConstants.LED_COUNT);
    }

    public void updateInputs(LedIOInputs inputs) {
        inputs.currentColor = buffer.getLED(0);
    }

    public void periodic() {
        currentPattern.applyTo(buffer);
    }

    public void setPattern(LEDPattern pattern) {
        currentPattern = pattern;
    }
}
