package frc.robot.subsystems.slapdown;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SlapdownConstants;

public class Slapdown extends SubsystemBase {
    
    private final SlapdownIOInputsAutoLogged inputs = new SlapdownIOInputsAutoLogged();
    private final SlapdownIO io;
    public final PIDController pid = new PIDController(SlapdownConstants.KP, SlapdownConstants.KI, SlapdownConstants.KD);
    public final ArmFeedforward feedforward = new ArmFeedforward(SlapdownConstants.KS, SlapdownConstants.KG, SlapdownConstants.KV);
    private final TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(SlapdownConstants.MAX_ANGLE_DEGREES, SlapdownConstants.MAX_ACCELERATION));
    private TrapezoidProfile.State goal = new TrapezoidProfile.State(0, 0);
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    public Slapdown(SlapdownIO io) {
        this.io = io;
    }

    public Command goToIntakeCommand() {
        return Commands.runOnce(
            () -> goal = new TrapezoidProfile.State(SlapdownConstants.INTAKE_ANGLE_DEGREES, 0)
        );
    }

    public Command goToHomeCommand() {
        return Commands.runOnce(
            () -> goal = new TrapezoidProfile.State(SlapdownConstants.HOME_ANGLE_DEGREES, 0)
        );
    }

    public Command outtakeCommand() {
        return Commands.runEnd(
            () -> io.runIntakeVoltage(-SlapdownConstants.INTAKE_VOLTAGE),
            () -> io.runIntakeVoltage(0)
        );
    }

    public Command intakeCommand() {
        return Commands.runEnd(
            () -> io.runIntakeVoltage(SlapdownConstants.INTAKE_VOLTAGE),
            () -> io.runIntakeVoltage(0)
        );
    }

    public Command stopIntakeCommand() {
        return Commands.runOnce(
            () -> io.runIntakeVoltage(0)
        );
    }

    // Double check with test
    public Command agitateCommand() {
        return intakeCommand().withTimeout(0.15).andThen(
            Commands.repeatingSequence(
                outtakeCommand().withTimeout(0.05),
                intakeCommand().withTimeout(0.25)
            ));
    }



    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("SlapdownIntake", inputs);

        if (DriverStation.isDisabled()) {
            setpoint = new TrapezoidProfile.State(inputs.absolutePosition, 0);
            goal = setpoint;
        }

        setpoint = profile.calculate(0.02, setpoint, goal);
        double voltage = pid.calculate(inputs.absolutePosition, setpoint.position) + 
            feedforward.calculate(inputs.absolutePosition + 90, setpoint.velocity);

        Logger.recordOutput("Slapdown/SetpointPosition", setpoint.position);
        Logger.recordOutput("Slapdown/GoalPosition", goal.position);
        Logger.recordOutput("Slapdown/Voltage", voltage);
        io.runPivotVoltage(voltage); 
    }

    public Command setAngleDegrees(double angle) {
        return runOnce(
            () -> goal = new TrapezoidProfile.State(angle, 0)
        );
    }
    
    public double getAngle() {
        return inputs.absolutePosition;
    }
}