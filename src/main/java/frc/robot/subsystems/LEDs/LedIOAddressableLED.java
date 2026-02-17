package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import frc.robot.Constants.LedConstants;

public class LedIOAddressableLED implements LedIO {
    private final AddressableLED led;
    private AddressableLEDBuffer buffer;
    private LEDPattern currentPattern;

    public LedIOAddressableLED() {
        led = new AddressableLED(LedConstants.PWM_PORT);
        buffer = new AddressableLEDBuffer(LedConstants.LED_COUNT);
        led.setLength(buffer.getLength());
    }
    
    public void updateInputs(LedIOInputs inputs) {
        inputs.currentColor = buffer.getLED(0);
    }

    public void periodic() {
        currentPattern.applyTo(buffer);
        led.setData(buffer);
    }

    public void setPattern(LEDPattern pattern) {
        currentPattern = pattern;
    }
    
    public void enable() {
        led.start();
    }

    public void disable() {
        led.stop();
    }
}