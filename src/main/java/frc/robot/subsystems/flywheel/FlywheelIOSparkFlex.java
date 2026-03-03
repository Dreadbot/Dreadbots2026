package frc.robot.subsystems.flywheel;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import frc.robot.Constants.FlywheelConstants;

public class FlywheelIOSparkFlex implements FlywheelIO {
    private final SparkFlex motor1;
    private final SparkFlex motor2;
    SparkClosedLoopController controller;

    public FlywheelIOSparkFlex() {
        motor1 = new SparkFlex(FlywheelConstants.MOTOR_ID_1, MotorType.kBrushless);
        SparkFlexConfig config1 = new SparkFlexConfig();
        config1
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(50);
        // config1.closedLoop
        //     .p(0.003)
        //     .i(0)
        //     .d(0)
        //     .outputRange(0, 1);
        motor1.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        controller = motor1.getClosedLoopController();
        motor2 = new SparkFlex(FlywheelConstants.MOTOR_ID_2, MotorType.kBrushless);
        SparkFlexConfig config2 = new SparkFlexConfig();
        config2.follow(FlywheelConstants.MOTOR_ID_1, true);
        motor2.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setVoltage(double volts) {
        motor1.setVoltage(volts);
    }

    @Override
    public void setRPM(double RPM) {
        controller.setSetpoint(RPM, ControlType.kVelocity);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.appliedVolts = motor1.getAppliedOutput() * motor1.getBusVoltage();
        inputs.RPM = motor1.getEncoder().getVelocity();
    }
}
