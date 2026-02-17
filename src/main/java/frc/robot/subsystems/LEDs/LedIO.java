package frc.robot.subsystems.LEDs;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public interface LedIO {
    @AutoLog
    public static class LedIOInputs {
        Color currentColor;
    }

    public default void updateInputs(LedIOInputs inputs) {};

    public default void setPattern(LEDPattern pattern) {};

    public default void periodic() {};

    public default void enable() {};

    public default void disable() {};
}