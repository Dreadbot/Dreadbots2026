package frc.robot.subsystems.turret;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.TurretConstants;

public class TurretIOSim implements TurretIO {
    
    private DCMotorSim turret;

    public TurretIOSim() {
        this.turret = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getNeo550(1), 
                TurretConstants.TURRET_MOI, 
                TurretConstants.GEAR_REDUCTION),
            DCMotor.getNeo550(1));
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        turret.update(0.02);
        inputs.turretAppliedVolts = 0.0;
        inputs.turretCurrentAmps = turret.getCurrentDrawAmps();
        inputs.turretRotationRad = turret.getAngularPositionRad();
    }

    @Override
    public void runTurretVoltage(double volts){
        turret.setInputVoltage(volts);
    }
}

