package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class Turret extends SubsystemBase {
    private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    private TurretIO io;
    private final PIDController pid = new PIDController(0.2, 0, 0.003);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.0, 0.00896);
    private double setpointRelativeRad;
    private final PIDController pid = new PIDController(1, 0, 0);
    private final ArmFeedforward feedforward = new ArmFeedforward(0.26, 0.15, 0.03);
    private final TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(540, 840));
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
    public double voltage;

    public Turret(TurretIO io) {
        this.io = io;
        this.voltage = 0;
        io.updateInputs(inputs);
        pid.setTolerance(Units.degreesToRadians(1));
    }

    @Override
    public void periodic() {
        setCorrectedAngleRad(setpointRelativeRad + Units.degreesToRadians(1));
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

        double wrappedSetpoint = wrapToLimits(setpointRelativeRad);
        
        voltage = pid.calculate(inputs.turretRotationRad, wrappedSetpoint)
         + feedforward.calculate(Units.degreesToRadians(50));

        if (inputs.turretRotationRad >= TurretConstants.MAX_ANGLE_RAD && voltage > 0) {
            voltage = 0.0;
        } else if (inputs.turretRotationRad <= TurretConstants.MIN_ANGLE_RAD && voltage < 0) {
            voltage = 0.0;
        }
        
        io.runTurretVoltage(voltage);

        Logger.recordOutput("Turret/SetpointRelativeRadians", wrappedSetpoint);
        Logger.recordOutput("Turret/CurrentPositionRadians", inputs.turretRotationRad);
        Logger.recordOutput("Turret/Delta", MathUtil.angleModulus(wrappedSetpoint - inputs.turretRotationRad));
        Logger.recordOutput("Turret/AtSetpoint", atSetpoint());
        Logger.recordOutput("Turret/Voltage", voltage);
    }   
    
    public Command setAngleRad(double angleRad) {
        return runOnce(
            () -> {
                setCorrectedAngleRad(angleRad); 
            } );
    }

    public void setCorrectedAngleRad(double angleRad) {
        double currentPositionWrapped = MathUtil.angleModulus(inputs.turretRotationRad);
        double delta = MathUtil.angleModulus(MathUtil.angleModulus(angleRad) - currentPositionWrapped);
        setpointRelativeRad = wrapToLimits(inputs.turretRotationRad + delta); 
                goal = new TrapezoidProfile.State(angle, 0);
            });
    }

    public Command setAtZero() {
        return runOnce(
            () -> {
                setpointRelativeRad = 0.0;
            } );
    }

    public static double wrapToLimits(double angleRad) {
        while (angleRad > TurretConstants.MAX_ANGLE_RAD) angleRad -= 2 * Math.PI;
        while (angleRad < TurretConstants.MIN_ANGLE_RAD) angleRad += 2 * Math.PI;
        return angleRad;
    }

    public double getAngle() {
        return inputs.turretRotationRad;
    }

    public boolean atSetpoint() {
        return MathUtil.isNear(setpointRelativeRad, inputs.turretRotationRad, Units.degreesToRadians(1));
    }
}