package frc.robot.subsystems.indexer;
// imports necessary libraries
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIO.IndexerIOInputs;


import frc.robot.Constants.IndexerConstants;

public class IndexerIOSparkFlex implements IndexerIO {
    private SparkFlex indexerMotor;
    private final SparkFlex kickerMotor;
    // initializes indexerMotor and kickerMotor
    public IndexerIOSparkFlex() {
        indexerMotor = new SparkFlex(IndexerConstants.MOTOR_ID, MotorType.kBrushless);
        this.kickerMotor = new SparkFlex(IndexerConstants.MOTOR_ID, MotorType.kBrushless);
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake).smartCurrentLimit(50);
        indexerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        kickerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    // updates the inputs
    @Override
    public void updateInputs(IndexerIOInputs inputs, IndexerIOInputs kickerInputs) {
        inputs.appliedVolts = indexerMotor.getAppliedOutput() * indexerMotor.getBusVoltage();
        inputs.currentAmps = indexerMotor.getOutputCurrent();
        inputs.RPM = indexerMotor.getEncoder().getVelocity();
        kickerInputs.appliedVolts = kickerMotor.getAppliedOutput() * kickerMotor.getBusVoltage();
        kickerInputs.currentAmps = kickerMotor.getOutputCurrent();
        kickerInputs.RPM = kickerMotor.getEncoder().getVelocity();
    }
    // runs voltage for the indexerMotor
    @Override
    public void runVoltage(double volts) {
        indexerMotor.setVoltage(volts);
    }
    // runs voltage for the kickerMotor
    @Override
    public void runKickerVoltage(double volts) {
        kickerMotor.setVoltage(volts);
    }
}
