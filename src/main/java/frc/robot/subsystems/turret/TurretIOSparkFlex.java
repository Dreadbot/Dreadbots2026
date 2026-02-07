package frc.robot.subsystems.turret;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.TurretConstants;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class TurretIOSparkFlex implements TurretIO {
    
    private final SparkFlex turretMotor;
    private DutyCycleEncoder absoluteEncoder;
    private double volts = 0.0;
//id is 14 but one for testing
   public TurretIOSparkFlex() {
       this.turretMotor = new SparkFlex(1, MotorType.kBrushless);
       this.absoluteEncoder = new DutyCycleEncoder(new DigitalInput(TurretConstants.TURRET_DUTY_CYCLE_ENCODER),
       TurretConstants.MAX_ANGLE_RAD, TurretConstants.TURRET_EXPECTED_ZERO);
       this.volts = 0.0;
       absoluteEncoder.setInverted(true);
       absoluteEncoder.setAssumedFrequency(975.6);
       SparkMaxConfig config = new SparkMaxConfig();
       config.idleMode(IdleMode.kBrake);
       turretMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }    
       
    @Override   
    public void updateInputs(TurretIOInputs inputs) {
        inputs.pivotAppliedVolts = turretMotor.getAppliedOutput() * turretMotor.getBusVoltage();
        inputs.pivotCurrentAmps = turretMotor.getOutputCurrent();
        inputs.turretRotationRad = (Units.rotationsToRadians(absoluteEncoder.get()) - TurretConstants.TURRET_ENCODER_OFFSET_RAD);
    }


    @Override
    public void runTurretVoltage(double volts){
        turretMotor.setVoltage(volts);
        this.volts = volts;
    }
}
