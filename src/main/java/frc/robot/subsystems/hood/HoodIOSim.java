package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class HoodIOSim implements HoodIO {

    private static final double kGearRatio = 100.0;
    private static final double kArmLengthMeters = 0.3;
    private static final double kArmMassKg = 2.0;

    private final SingleJointedArmSim sim = 
        new SingleJointedArmSim(
            DCMotor.getNeo550(1),
            kGearRatio,
            SingleJointedArmSim.estimateMOI(kArmLengthMeters, kArmMassKg),
            kArmLengthMeters,
            Units.degreesToRadians(10), // min angle
            Units.degreesToRadians(45), // max angle
            true, // simulate gravity
            Units.degreesToRadians(10) // initial angle
        );

    private double appliedVolts = 0.0;

    @Override
    public void setVoltage(double volts) {
        appliedVolts = volts;
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        sim.setInputVoltage(appliedVolts);
        sim.update(0.02);

        inputs.angleDeg = Units.radiansToDegrees(sim.getAngleRads());

        inputs.velocityDegPerSec = Units.radiansToDegrees(sim.getVelocityRadPerSec());

        inputs.appliedVolts = appliedVolts;
    }
}
