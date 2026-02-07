package frc.robot.subsystems.turret;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.TurretConstants;

public class TurretIOSim implements TurretIO {
    
    private SingleJointedArmSim turret;

    public TurretIOSim() {
        this.turret = new SingleJointedArmSim(DCMotor.getNeo550(1), 
        TurretConstants.GEAR_REDUCTION, 
        0.0366, 
        0.075, 
        TurretConstants.MIN_ANGLE_RAD, 
        TurretConstants.MAX_ANGLE_RAD, 
        false,
        0.0
        );
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        turret.update(0.02);
        inputs.turretAppliedVolts = 0.0;
        inputs.turretRPM = (turret.getVelocityRadPerSec());
        inputs.turretCurrentAmps = turret.getCurrentDrawAmps();
        inputs.turretRotationRad = Units.radiansToDegrees(turret.getAngleRads());
    }

    @Override
    public void runTurretVoltage(double volts){
        turret.setInputVoltage(volts);
    }
}

