package frc.robot.subsystems.slapdown;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.SlapdownConstants;


public class SlapdownIOSparkFlex implements SlapdownIO {

    private final SparkBase intakeMotor;
    private final SparkBase pivotMotor;
    private final DutyCycleEncoder absoluteEncoder;

    public SlapdownIOSparkFlex() {
        this.absoluteEncoder = new DutyCycleEncoder(new DigitalInput(SlapdownConstants.SLAPDOWN_DUTY_CYCLE_ENCODER), 360.0, 0.0); //Update code with the 0 and max angle
        absoluteEncoder.setAssumedFrequency(SlapdownConstants.ENCODER_FREQUENCY);
        absoluteEncoder.setInverted(true);
        this.intakeMotor = new SparkFlex(SlapdownConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
        this.pivotMotor = new SparkFlex(SlapdownConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);
        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        SparkMaxConfig pivotConfig = new SparkMaxConfig();

        intakeConfig
            .idleMode(IdleMode.kCoast)
            .inverted(true);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivotConfig
            .idleMode(IdleMode.kBrake)
            .inverted(false);
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

        @Override
        public void updateInputs(SlapdownIOInputs inputs) {
            inputs.absolutePosition = absoluteEncoder.get() - SlapdownConstants.ENCODER_OFFSET;
            inputs.intakeRPM = intakeMotor.getEncoder().getVelocity();

            // inputs.intakeAppliedVolts = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
            // inputs.intakeCurrentAmps = intakeMotor.getOutputCurrent();

            inputs.pivotAppliedVolts = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
            //inputs.pivotCurrentAmps = pivotMotor.getOutputCurrent();

            //inputs.pivotRotationDegrees = absoluteEncoder.get();
        }

        @Override
        public void runIntakeVoltage(double voltage) {
            intakeMotor.setVoltage(voltage);
        }

        @Override
        public void setIdleMode(IdleMode pivotIdleMode, IdleMode intakeIdleMode) {
            SparkMaxConfig pivotConfig = new SparkMaxConfig();
            pivotConfig.idleMode(pivotIdleMode);
            SparkMaxConfig intakeConfig = new SparkMaxConfig();
            intakeConfig.idleMode(intakeIdleMode);
            /* 
            * Don't reset parameters + don't save this config if reboot hapens. 
            * This is ony if we need to switch out of break mode for some reason
            */ 
            intakeMotor.configure(intakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            pivotMotor.configure(pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }

        @Override
        public void stopMotors() {
            intakeMotor.setVoltage(0);
            intakeMotor.stopMotor();
            pivotMotor.setVoltage(0);
            pivotMotor.stopMotor();
        }

        @Override
        public void runPivotVoltage(double voltage){
            pivotMotor.setVoltage(voltage);
        }
    }