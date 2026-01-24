package frc.robot.subsystems.flywheel;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class FlywheelIOSparkFlex implements FlywheelIO {

    private final SparkFlex motor;
    private final RelativeEncoder encoder;

    private double appliedVolts = 0.0;

    public FlywheelIOSparkFlex() {
        motor = new SparkFlex(1, MotorType.kBrushless);
        encoder = motor.getEncoder();
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = volts;
        motor.setVoltage(volts);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.velocityRPM = encoder.getVelocity();
        inputs.appliedVolts = appliedVolts;
    }
}
