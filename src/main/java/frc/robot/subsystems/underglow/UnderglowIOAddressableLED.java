package frc.robot.subsystems.underglow;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import frc.robot.Constants.UnderglowConstants;

public class UnderglowIOAddressableLED implements UnderglowIO {
    private final AddressableLED led;
    private AddressableLEDBuffer buffer;
    private LEDPattern currentPattern;

    public UnderglowIOAddressableLED() {
        led = new AddressableLED(UnderglowConstants.PWM_PORT);
        buffer = new AddressableLEDBuffer(UnderglowConstants.LED_COUNT);
        led.setLength(buffer.getLength());
    }
    
    public void updateInputs(UnderglowIOInputs inputs) {
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