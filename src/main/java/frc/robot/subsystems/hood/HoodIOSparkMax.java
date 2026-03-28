package frc.robot.subsystems.hood;

import com.revrobotics.PersistMode;
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
    // Constructor initializes the SparkMax motor controller and the lower limit switch, and configures the motor with specific settings for idle mode, inversion, and current limit to ensure safe and effective operation of the hood mechanism.
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
    // Method to set the motor voltage, which is used to control the speed and direction of the hood mechanism. This method is called by the Hood subsystem to apply the calculated voltage from the PID controller and feedforward to move the hood towards the desired position.
    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }
    // Method to set the position of the hood, which is called when the lower limit switch is triggered during calibration to reset the hood's position to 0. This allows the Hood subsystem to have a known reference point for controlling the hood's position accurately.
    @Override
    public void setPosition(double position) {
        motor.getEncoder().setPosition(position);
    }
    // Method to update the inputs for the Hood subsystem, which reads the current applied voltage, the encoder position (in rotations), and the state of the lower limit switch. This information is used by the Hood subsystem to determine the current state of the hood mechanism and to make decisions about how to control it.
    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.rotations = motor.getEncoder().getPosition();
        inputs.lowerSwitch = !lowerLimitSwitch.get();
    }
}