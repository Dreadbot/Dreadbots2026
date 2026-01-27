package frc.robot.subsystems.flywheel;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import frc.robot.Constants.FlywheelConstants;

public class FlywheelIOSparkFlex implements FlywheelIO {

    private final SparkFlex motor1;
    private final SparkFlex motor2;

    private double appliedVolts = 0.0;

    public FlywheelIOSparkFlex() {
        motor1 = new SparkFlex(FlywheelConstants.MOTOR_ID_1, MotorType.kBrushless);
        SparkFlexConfig config1 = new SparkFlexConfig();
        config1
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50);
        motor1.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor2 = new SparkFlex(FlywheelConstants.MOTOR_ID_2, MotorType.kBrushless);
        SparkFlexConfig config2 = new SparkFlexConfig();
        config2.follow(FlywheelConstants.MOTOR_ID_1);
        motor2.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = volts;
        motor1.setVoltage(volts);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.velocityRPM = motor1.getEncoder().getVelocity();
        inputs.appliedVolts = appliedVolts;
    }
}
