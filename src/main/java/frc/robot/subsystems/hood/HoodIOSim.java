package frc.robot.subsystems.hood;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HoodIOSim implements HoodIO {
    private final DCMotorSim hood;
    
    public HoodIOSim() {
        hood = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getNEO(1),
                0.0366,
                150.0
            ),
            DCMotor.getNeo550(1)
        );
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        hood.update(0.02);
        inputs.RPM = hood.getAngularVelocityRPM();
        inputs.appliedVolts = hood.getInputVoltage();
        inputs.angle = hood.getAngularPositionRad();
    }

    @Override 
    public void setVoltage(double volts) {
        hood.setInputVoltage(volts);
    }
}