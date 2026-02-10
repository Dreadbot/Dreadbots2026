package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {

    @AutoLog
    public static class HoodIOInputs {
        public double angle = 0.0;
        public double velocity = 0.0;
        public double appliedVolts = 0.0;
    }

    default void updateInputs(HoodIOInputs inputs) {}

    default void setVoltage(double volts) {}

    default void setPosition(double position) {}
}
