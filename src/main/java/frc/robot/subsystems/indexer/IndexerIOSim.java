package frc.robot.subsystems.indexer;
// Imports necessary libraries
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndexerIOSim implements IndexerIO {
    // Sets up motor simulation
    private final DCMotorSim motorSim;
    private final double rollerWheelMOI = 0.5 * Units.lbsToKilograms(0.12) * Units.inchesToMeters(1.5) * Units.inchesToMeters(1.5);
    public IndexerIOSim() {
        this.motorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 3 * rollerWheelMOI, 1.0),
            DCMotor.getNeoVortex(1)
        );
    }

    // Updates the inputs for the simulation
    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        motorSim.update(0.02);
        inputs.appliedVolts = 0.0;
        inputs.RPM = motorSim.getAngularVelocityRPM();
        inputs.currentAmps = motorSim.getCurrentDrawAmps();
    }

    // Runs voltage for the simulation
    @Override
    public void runVoltage(double volts) {
        motorSim.setInputVoltage(volts);
    }
}
