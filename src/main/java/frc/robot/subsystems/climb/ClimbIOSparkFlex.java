package frc.robot.subsystems.climb;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants.ClimbConstants;

public class ClimbIOSparkFlex implements ClimbIO {
    private SparkFlex screwMotor;

    public ClimbIOSparkFlex() {
        screwMotor = new SparkFlex(ClimbConstants.MOTOR_ID, MotorType.kBrushless);
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake).smartCurrentLimit(50);
        screwMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void updateInputs(ClimbIOInputs inputs) {
        inputs.appliedVolts = screwMotor.getAppliedOutput() * screwMotor.getBusVoltage();
        inputs.currentAmps = screwMotor.getOutputCurrent();
        inputs.RPM = screwMotor.getEncoder().getVelocity();
    }

    public void runVoltage(double volts) {
        screwMotor.setVoltage(volts);
    }
}