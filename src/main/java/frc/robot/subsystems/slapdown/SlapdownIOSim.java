package frc.robot.subsystems.slapdown;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.SlapdownConstants;

public class SlapdownIOSim implements SlapdownIO {

    private final SingleJointedArmSim slapdown;
    private final DCMotorSim intakeMotor;

    private double pivotVolts;
    private double intakeVolts;


    public SlapdownIOSim() {
        this.slapdown = new SingleJointedArmSim(
            DCMotor.getNeoVortex(1), 
            SlapdownConstants.SLAPDOWN_GEARING, 
            SlapdownConstants.SIM_PIVOT_MOI, 
            SlapdownConstants.ARM_LENGTH, 
            SlapdownConstants.MIN_ANGLE_RAD, 
            SlapdownConstants.MAX_ANGLE_RAD, 
            true,
            Units.degreesToRadians(0)
            );
        this.intakeMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), SlapdownConstants.SIM_INTAKE_MOI, SlapdownConstants.INTAKE_GEARING), 
            DCMotor.getNEO(1)
        );
        pivotVolts = 0.0;
        intakeVolts = 0.0;

    }

    @Override
    public void updateInputs(SlapdownIOInputs inputs) {
        slapdown.update(0.02);
        intakeMotor.update(0.02);

        inputs.pivotAppliedVolts = pivotVolts;
        inputs.intakeAppliedVolts = intakeVolts;
       
        inputs.pivotCurrentAmps = slapdown.getCurrentDrawAmps();
        inputs.intakeCurrentAmps = intakeMotor.getCurrentDrawAmps();

        inputs.pivotRotationDegrees = Units.radiansToDegrees(slapdown.getAngleRads());

        inputs.intakeRPM = intakeMotor.getAngularVelocityRPM();
    } 

    @Override
    public void runPivotVoltage(double volts) {
        slapdown.setInputVoltage(volts);
        this.pivotVolts = volts;
    }
    @Override
    public void runIntakeVoltage(double volts) {
        intakeMotor.setInputVoltage(volts);
        this.intakeVolts = volts;
    }

}