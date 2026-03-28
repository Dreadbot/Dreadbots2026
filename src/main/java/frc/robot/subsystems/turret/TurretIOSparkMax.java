package frc.robot.subsystems.turret;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.flywheel.Flywheel;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class TurretIOSparkMax implements TurretIO {
    
    private final SparkMax turretMotor;
    private DutyCycleEncoder absoluteEncoder;
    private double volts = 0.0;
    
    public TurretIOSparkMax() {
        this.turretMotor = new SparkMax(TurretConstants.TURRET_MOTOR_ID, MotorType.kBrushless);

        this.absoluteEncoder = new DutyCycleEncoder(
            new DigitalInput(TurretConstants.TURRET_DUTY_CYCLE_ENCODER),
            2 * Math.PI, 
            0.0);
        absoluteEncoder.setInverted(true);
        absoluteEncoder.setAssumedFrequency(975.6);
        
        this.volts = 0.0;
        
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast);

        turretMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        double absoluteRad = MathUtil.angleModulus(absoluteEncoder.get() - Math.PI - TurretConstants.TURRET_ENCODER_OFFSET_RAD);
        double motorRotations = Units.radiansToRotations(absoluteRad) * TurretConstants.GEAR_REDUCTION;
        turretMotor.getEncoder().setPosition(motorRotations);
    }

    @Override
    public void setZero() {
        turretMotor.getEncoder().setPosition(0.0);
    }
       
    @Override   
    public void updateInputs(TurretIOInputs inputs) {
        inputs.turretAppliedVolts = turretMotor.getAppliedOutput() * turretMotor.getBusVoltage();
        inputs.turretCurrentAmps = turretMotor.getOutputCurrent();
        inputs.turretRotationRad = Units.rotationsToRadians(turretMotor.getEncoder().getPosition()) / TurretConstants.GEAR_REDUCTION;
        inputs.absolutePosition = absoluteEncoder.get() - Math.PI;
        inputs.turretVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(turretMotor.getEncoder().getVelocity())
    / TurretConstants.GEAR_REDUCTION;
    }

    @Override
    public void runTurretVoltage(double volts){
        turretMotor.setVoltage(volts);
        this.volts = volts;
    }
}
