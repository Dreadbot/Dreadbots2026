package frc.robot.subsystems.turret;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class TurretIOSim implements TurretIO {
    
    private SingleJointedArmSim turret;

    public TurretIOSim() {
        this.turret = new SingleJointedArmSim(DCMotor.getNEO(1), 
        50.0, 
        SingleJointedArmSim.estimateMOI(0.15, Units.lbsToKilograms(10)), 
        0.15, 
        Units.degreesToRadians(-90), 
        Units.degreesToRadians(90), 
        true,
        Units.degreesToRadians(0)
        );
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        turret.update(0.02);
        inputs.pivotAppliedVolts = 0.0;
        inputs.pivotRPM = (turret.getVelocityRadPerSec());
        inputs.pivotCurrentAmps = turret.getCurrentDrawAmps();
        inputs.pivotRotationDegrees = Units.radiansToDegrees(turret.getAngleRads());
    }

    @Override
    public void runPivotVoltage(double volts){
        turret.setInputVoltage(volts);
    }
}

