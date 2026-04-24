package frc.robot.subsystems.slapdown;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.SlapdownConstants;


public class SlapdownIOSparkFlex implements SlapdownIO {

    private final TalonFX intakeMotor;
    private final SparkBase pivotMotor;
    private final VoltageOut voltageOut;
    private final DutyCycleEncoder absoluteEncoder;

    public SlapdownIOSparkFlex() {
        this.absoluteEncoder = new DutyCycleEncoder(new DigitalInput(SlapdownConstants.SLAPDOWN_DUTY_CYCLE_ENCODER), 360.0, -SlapdownConstants.ENCODER_ZERO); //Update code with the 0 and max angle
        absoluteEncoder.setAssumedFrequency(SlapdownConstants.ENCODER_FREQUENCY);
        absoluteEncoder.setInverted(true);
        this.intakeMotor = new TalonFX(SlapdownConstants.INTAKE_MOTOR_ID);
        this.pivotMotor = new SparkFlex(SlapdownConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);
        this.voltageOut = new VoltageOut(0);
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        SparkMaxConfig pivotConfig = new SparkMaxConfig();

        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // intakeConfig.CurrentLimits.StatorCurrentLimit = 80;
        // intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeMotor.getConfigurator().apply(intakeConfig);

        pivotConfig
            .idleMode(IdleMode.kBrake)
            .inverted(false);
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

        @Override
        public void updateInputs(SlapdownIOInputs inputs) {
            inputs.absolutePosition = absoluteEncoder.get() - SlapdownConstants.ENCODER_OFFSET;
            inputs.intakeRPM = intakeMotor.getVelocity().getValueAsDouble();

            inputs.pivotAppliedVolts = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
        }

        @Override
        public void runIntakeVoltage(double voltage) {
            intakeMotor.setControl(voltageOut.withOutput(voltage).withEnableFOC(true));
        }

        @Override
        public void setIdleMode(IdleMode pivotIdleMode, IdleMode intakeIdleMode) {
            SparkMaxConfig pivotConfig = new SparkMaxConfig();
            pivotConfig.idleMode(pivotIdleMode);
            //SparkMaxConfig intakeConfig = new SparkMaxConfig();
            //intakeConfig.idleMode(intakeIdleMode);
            /* 
            * Don't reset parameters + don't save this config if reboot hapens. 
            * This is ony if we need to switch out of break mode for some reason
            */ 
            //intakeMotor.configure(intakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            pivotMotor.configure(pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }

        @Override
        public void stopMotors() {
            runIntakeVoltage(0);
            intakeMotor.stopMotor();
            runPivotVoltage(0);
            pivotMotor.stopMotor();
        }

        @Override
        public void runPivotVoltage(double voltage){
            pivotMotor.setVoltage(voltage);
        }
    }