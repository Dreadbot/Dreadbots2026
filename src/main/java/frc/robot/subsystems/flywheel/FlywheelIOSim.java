package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.util.Units;

public class FlywheelIOSim implements FlywheelIO {

    private static final LinearSystem<N1, N1, N1> flywheelPlant = 
        LinearSystemId.createFlywheelSystem(
            DCMotor.getNeoVortex(2),
            0.001,
            1.0
        );

    private final FlywheelSim sim = 
        new FlywheelSim(
            flywheelPlant,
            DCMotor.getNeoVortex(2),
            0.0
        );

    private double appliedVolts = 0.0;

    @Override
    public void setVoltage(double volts) {
        appliedVolts = volts;
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        sim.setInputVoltage(appliedVolts);
        sim.update(0.02);

        inputs.velocityRPM = Units.radiansPerSecondToRotationsPerMinute(sim.getAngularVelocityRadPerSec());

        inputs.appliedVolts = appliedVolts;
    }
}
