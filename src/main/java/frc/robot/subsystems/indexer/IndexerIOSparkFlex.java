package frc.robot.subsystems.indexer;
// imports necessary libraries
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.IndexerConstants;

public class IndexerIOSparkFlex implements IndexerIO {
    private final SparkFlex indexerMotor;
    private final SparkFlex kickerMotor;
    // initializes indexerMotor and kickerMotor
    public IndexerIOSparkFlex() {
        this.indexerMotor = new SparkFlex(IndexerConstants.MOTOR_ID, MotorType.kBrushless);
        this.kickerMotor = new SparkFlex(IndexerConstants.MOTOR_ID + 1, MotorType.kBrushless); // change the MOTOR_ID + 1 to the actual ID of the kicker motor
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake).smartCurrentLimit(50);
        indexerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        kickerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    // updates the inputs
    public void updateInputs(IndexerIOInputs indexerInputs, IndexerIOInputs kickerInputs) {
        indexerInputs.appliedVolts = indexerMotor.getAppliedOutput() * indexerMotor.getBusVoltage();
        indexerInputs.currentAmps = indexerMotor.getOutputCurrent();
        indexerInputs.RPM = indexerMotor.getEncoder().getVelocity();
        kickerInputs.appliedVolts = kickerMotor.getAppliedOutput() * kickerMotor.getBusVoltage();
        kickerInputs.currentAmps = kickerMotor.getOutputCurrent();
        kickerInputs.RPM = kickerMotor.getEncoder().getVelocity();
    }
    // runs voltage for the indexerMotor and kickerMotor
    public void runVoltage(double indexerVolts, double kickerVolts) {
        indexerMotor.setVoltage(indexerVolts);
        kickerMotor.setVoltage(kickerVolts);
    }
}
