package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;

import java.util.Optional;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import static frc.robot.Constants.TurretConstants.*;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class TurretIOSparkMax implements TurretIO {
    
    private final SparkMax turretMotor;
    private DutyCycleEncoder absoluteEncoder1;
    private DutyCycleEncoder absoluteEncoder2;
    private EasyCRT easyCRT;
    
    public TurretIOSparkMax() {
        this.turretMotor = new SparkMax(MOTOR_ID, MotorType.kBrushless);

        this.absoluteEncoder1 = new DutyCycleEncoder(
                new DigitalInput(ENCODER_1_PORT),
                1.0, 
                0.0);
            absoluteEncoder1.setInverted(ENCODER_1_INVERTED);
            absoluteEncoder1.setAssumedFrequency(975.6);
        this.absoluteEncoder2 = new DutyCycleEncoder(
                new DigitalInput(ENCODER_2_PORT),
                1.0, 
                0.0);
            absoluteEncoder2.setInverted(ENCODER_2_INVERTED);
            absoluteEncoder2.setAssumedFrequency(975.6);
        
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast);

        turretMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        EasyCRTConfig crtConfig =
            new EasyCRTConfig(this::encoder1Supplier, this::encoder2Supplier)
                .withCommonDriveGear(
                    /* commonRatio (mech:drive) */ MECH_TO_DRIVE,
                    /* driveGearTeeth */ DRIVE_GEAR_TEETH,
                    /* encoder1Pinion */ ENCODER_1_TEETH,
                    /* encoder2Pinion */ ENCODER_2_TEETH)
                .withAbsoluteEncoderOffsets(Rotations.of(-ENCODER_1_ZERO), Rotations.of(-ENCODER_2_ZERO)) // set after mechanical zero
                .withMechanismRange(Rotations.of(-2.0), Rotations.of(2.0))
                .withMatchTolerance(Rotations.of(0.06)) // ~1.08 deg at encoder2 for the example ratio
                .withAbsoluteEncoderInversions(ENCODER_1_INVERTED, ENCODER_2_INVERTED)
                .withCrtGearRecommendationConstraints(
                    /* coverageMargin */ 1.2,
                    /* minTeeth */ 15,
                    /* maxTeeth */ 45,
                    /* maxIterations */ 30);
        
        this.easyCRT = new EasyCRT(crtConfig);

        //double absoluteRad = MathUtil.angleModulus(absoluteEncoder.get() - Math.PI - TurretConstants.TURRET_ENCODER_OFFSET_RAD);
        Optional<Angle> absoluteAngle = solveAngle();
        absoluteAngle.ifPresent(angle -> {
            turretMotor.getEncoder().setPosition(angle.in(Rotations) * GEAR_REDUCTION);
        });
        if (!absoluteAngle.isPresent()) {
            System.err.println("Turret CRT could not solve on bootup.");
        }
    }

    private Angle encoder1Supplier() {
        return Rotations.of(absoluteEncoder1.get());
    }
    private Angle encoder2Supplier() {
        return Rotations.of(absoluteEncoder2.get());
    }

    public Optional<Angle> solveAngle() {
        return easyCRT.getAngleOptional();
    }

    @Override
    public void setZero() {
        turretMotor.getEncoder().setPosition(0.0);
    }
       
    @Override   
    public void updateInputs(TurretIOInputs inputs) {
        inputs.turretAppliedVolts = turretMotor.getAppliedOutput() * turretMotor.getBusVoltage();
        inputs.turretCurrentAmps = turretMotor.getOutputCurrent();
        inputs.turretRotationRad = Units.rotationsToRadians(turretMotor.getEncoder().getPosition()) / GEAR_REDUCTION;
        inputs.absoluteEncoder1 = absoluteEncoder1.get();
        inputs.absoluteEncoder2 = absoluteEncoder1.get();
        
        solveAngle().ifPresentOrElse(angle -> {
            System.out.println("Solved Angle: " + angle);
        }, () -> {
            System.out.println("CRT Could not solve");
        });
    }

    @Override
    public void runTurretVoltage(double volts){
        turretMotor.setVoltage(volts);
    }
}
