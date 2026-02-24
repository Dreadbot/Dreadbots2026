package frc.robot.subsystems.hood;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.HoodConstants;

public class HoodIOSparkMax implements HoodIO {
    private final SparkMax motor;
    private final DigitalInput lowerLimitSwitch;

    public HoodIOSparkMax() {
        motor = new SparkMax(HoodConstants.MOTOR_ID, MotorType.kBrushless);
        lowerLimitSwitch = new DigitalInput(HoodConstants.LOWER_LIMIT_SWITCH_ID);
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .idleMode(IdleMode.kCoast)
            .inverted(true)
            .smartCurrentLimit(50);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void setPosition(double position) {
        motor.getEncoder().setPosition(position);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.rotations = motor.getEncoder().getPosition();
        inputs.lowerSwitch = !lowerLimitSwitch.get();
    }
}