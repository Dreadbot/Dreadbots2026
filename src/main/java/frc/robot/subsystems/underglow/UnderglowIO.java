package frc.robot.subsystems.underglow;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public interface UnderglowIO {
    @AutoLog
    public static class UnderglowIOInputs {
        Color currentColor;
    }

    public default void updateInputs(UnderglowIOInputs inputs) {};

    public default void setPattern(LEDPattern pattern) {};

    public default void periodic() {};

    public default void enable() {};

    public default void disable() {};
}