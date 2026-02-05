package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

public class Turret extends SubsystemBase {
    private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    private TurretIO io;
    private final PIDController pid = new PIDController(0.1, 0, 0);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.0, 0.0);
    private final TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(540, 840));
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
    public DoubleSupplier joystickOverride;
    public double voltage;


    public Turret(TurretIO io) {
        this.io = io;
        this.joystickOverride = () -> 0.0;
        this.voltage = 0;
        io.updateInputs(inputs);
        goal = new TrapezoidProfile.State(inputs.pivotRotationDegrees, 0);
        setpoint = goal;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

         if (DriverStation.isDisabled()) {
             setpoint = new TrapezoidProfile.State(inputs.pivotRotationDegrees, 0);
             goal = setpoint;
         }

         Logger.recordOutput("Turret/SetpointPosition", setpoint.position);
         Logger.recordOutput("Turret/GoalPosition", goal.position);
         Logger.recordOutput("Turret/AtSetpoint", atSetpoint());
         setpoint = profile.calculate(0.02, setpoint, goal);
         voltage = pid.calculate(inputs.pivotRotationDegrees, setpoint.position)
         + feedforward.calculate(Units.degreesToRadians(setpoint.position) ,setpoint.velocity);
         io.runPivotVoltage(voltage);
    }   
    
    public Command setAngleDegrees(double angle) {
        return runOnce(
            () -> {
                goal = new TrapezoidProfile.State(angle, 0);
            } );
    }

    public Command setAtZero() {
        return runOnce(
            () -> {

            } );
    }

     public double getAngle() {
         return inputs.pivotRotationDegrees;
     }

     public boolean atSetpoint() {
         return MathUtil.isNear(goal.position, inputs.pivotRotationDegrees, 0.0);
     }
}