// package frc.robot.subsystems.turret;
// public class TurretIOSim implements TurretIO {
    
//         private final SingleJointedArmSim singleJointedArmSim;
//         private double volts;

//(?)         private double pivotVolts;
//(?)         private double intakeVolts;


//         public SlapdownAlgaeIOSIM() {
//             this.turret = new SingleJointedArmSim(DCMotor.getNeo550(1),
//                 TurretConstants.GEARING,
//                 TurretConstants.TURRET_MASS,
//                 TurretConstants.MIN_ANGLE,
//                 TurretConstants.MAX_ANGlE,
//                 true,
//                 TurretConstants.STARTING_HEIGHT
//             );
//             this.volts = 0.0;
//         }

//     @Override
//     public void updateInputs(TurretIOInputs inputs) {
//         pivotMotor.update(0.02);
//         intakeMotor.update(0.02);

//         inputs.pivotAppliedVolts = pivotVolts;

//         inputs.pivotCurrentAmps = pivotMotor.getCurrentDrawAmps();

//         inputs.pivotRotationDegrees = Units.radiansToDegrees(turret.getAngleRads());

//         inputs.intakeRPM = intakeMotor.getAngularVelocityRPM();
//         inputs.absolutePosition = pivotMotor.getAbsolutePosition

//     }

//     @Override
//     public void runPivotVoltage(double voltage){
//         pivotMotor.setInputVoltage(volts);
//         this.volts = volts;
//     }
// }

//     @Override
//     public void runIntakeVoltage(double voltage) {
//         intakeMotor.setInputVoltage(volts);
//         this.volts = volts;
//     }

//     @Override
//     public void setIdleMode(IdleMode pivotIdleMode, IdleMode intakeIdleMode) {
//         SparkMaxConfig pivotConfig = new SparkMaxConfig();
//         pivotConfig.idleMode(pivotIdleMode);
//         SparkMaxConfig intakeConfig = new SparkMaxConfig();
//         intakeConfig.idleMode(intakeIdleMode)
//         intakeMotor.configure(intakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
//         pivotMotor.configure(pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
//     }

//     @Override
//     public void stopMotors() {
//         intakeMotor.setVoltage();
//         intakeMotor.stopMotor();
//         pivotMotor.setVoltage();
//         pivotMotor.stopMotor();
//     }
//

