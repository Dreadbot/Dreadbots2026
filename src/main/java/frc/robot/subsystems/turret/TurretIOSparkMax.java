//package frc.robot.subsystems.turret;

//public class TurretIOSparkMax implements TurretIO {
    
//    private final SparkBase intakeMotor;
//    private final SparkBase pivotMotor;
//    private final DutyCycleEncoder absoluteEncoder;

//    public TurretIOSparkMax() {
//        public TurretIOSparkMax() {
//         this.absoluteEncoder = new DutyCycleEncoder(new DigitalInput(TurretConstants.TURRET_DUTY_CYCLE_ENCODER), 0, 0); //Update code with the 0 and max angle
//         absoluteEncoder.setAssumedFrequency(TurretConstants.ENCODER_FREQUENCY);
//         this.intakeMotor = new SparkMax(TurretConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
//         this.pivotMotor = new SparkMax(TurretConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);
//         SparkMaxConfig intakeConfig = new SparkMaxConfig();
//         SparkMaxConfig pivotConfig = new SparkMaxConfig();
//         
//         intakeConfig
//             .idleMode(IdleMode.kBrake);
//         intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameter);
//         pivotConfig
//             .idleMode(IdleMode.kBrake);
//         intakeMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameter);
//         }
//    }
//}
