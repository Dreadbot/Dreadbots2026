package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.slapdown.Slapdown;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;

public class Climb extends SubsystemBase {
    // Auto logging output to something
    private ClimbIO io;
    private ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    // PID work?
    private final PIDController pid = new PIDController(ClimbConstants.PIDCONTROLLER_KP,
            ClimbConstants.PIDCONTROLLER_KI,
            ClimbConstants.PIDCONTROLLER_KD);
    private final TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
            ClimbConstants.TRAPEZOID_CONSTRAINTS_MAX_VELOCITY, ClimbConstants.TRAPEZOID_CONSTRAINTS_MAX_ACCELERATION));
    private TrapezoidProfile.State goal = new TrapezoidProfile.State(ClimbConstants.TRAPEZOID_STATE_POSITION,
            ClimbConstants.TRAPEZOID_STATE_VELOCITY);
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
    public final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ClimbConstants.ARMFEEDFORWARD_KS,
            ClimbConstants.ARMFEEDFORWARD_KV);
    private DigitalInput lowerSwitch = new DigitalInput(ClimbConstants.LOWER_LIMIT_SWITCH_ID);
    private DigitalInput upperSwitch = new DigitalInput(ClimbConstants.UPPER_LIMIT_SWITCH_ID);

    @AutoLogOutput
    // Setting up the boolean Varible, which is for right now isClimbed (Basic will
    // be updated later)
    public boolean isClimbed = false;
    public boolean raisingArm = false;
    public boolean loweringArm = false;
    public boolean climbing = false;

    public Climb(ClimbIO io, Slapdown slapdown) {
        this.io = io;
    }

    // public Command motorForward() {
    // return Commands.startEnd(
    // () -> {
    // if (upperSwitch.get()) {
    // raisingArm = true;
    // }
    // },
    // () -> raisingArm = false);
    // }

    // public Command motorBackward() {
    // return Commands.startEnd(
    // () -> {
    // if (lowerSwitch.get()) {
    // loweringArm = true;
    // }
    // },
    // () -> loweringArm = false);
    // }

    // public Command raiseClimbArm() {
    // return Commands.runOnce(() -> {
    // if (upperSwitch.get())
    // raisingArm = true;
    // });
    // }

    // public Command levelOneClimb() {
    // return Commands.startEnd(
    // () -> {
    // io.runVoltage(ClimbConstants.RAISE_VOLTAGE);
    // },

    // () -> io.runVoltage(0.0),
    // this

    // );
    // }

    public Command lowerClimbArm() {
        return Commands.runOnce(() -> {
            // if (lowerSwitch.get()) {
            goal = new TrapezoidProfile.State(ClimbConstants.STOW_POSITION, 0);
            // loweringArm = true;
            // }
        });
    }

    public Command climb() {
        return Commands.runOnce(() -> {
            // if (lowerSwitch.get()) {
            goal = new TrapezoidProfile.State(ClimbConstants.CLIMB_POSITION, 0);
            // loweringArm = true;
            // }
        });
    }

    public Command raiseRobotLevelOne() {
        return Commands.runOnce(() -> {
            // if (lowerSwitch.get()) {
            goal = new TrapezoidProfile.State(ClimbConstants.LEVEL_ONE_CLIMB_POSITION, 0);
            // raisingArm = true;
            // }
        });
    }
    // public Command climb() {
    // return Commands.runOnce(() -> {
    // climbing = !climbing;
    // if (climbing) {
    // raisingArm = true;
    // }
    // else {
    // raisingArm = false;
    // }
    // loweringArm = false;
    // });
    // }

    public Command testLower() {
        return Commands.runEnd(
                () -> io.runVoltage(ClimbConstants.LOWER_VOLTAGE),
                () -> io.runVoltage(0.0),
                this);

    }

    public Command testRaise() {
        return Commands.runEnd(
                () -> io.runVoltage(ClimbConstants.RAISE_VOLTAGE),
                () -> io.runVoltage(0.0),
                this);
    }

    // public Command extentionChecker() {
    // return Commands.runOnce(() -> {
    // io.isSlapdownExtended();
    // });
    // }

    // public Command extentionChecker() {
    // return Commands.runOnce(() -> {
    // io.isSlapdownExtended();
    // });
    // }

    // Updates the inputs of ClimbIO perodic.
    // ClimbIO takes the inputs and outputs of Climb from the contorller
    @Override
    public void periodic() {

        io.updateInputs(inputs);
        Logger.processInputs("Climb", inputs);
        if (DriverStation.isDisabled()) {
            setpoint = new TrapezoidProfile.State(inputs.absolutePosition, 0);
            goal = setpoint;
        }

        Logger.recordOutput("Climb/SetpointPosition", setpoint.position);
        Logger.recordOutput("Climb/GoalPosition", goal.position);
        Logger.recordOutput("Climb/LowerSwitch", !lowerSwitch.get());
        Logger.recordOutput("Climb/UpperSwitch", !upperSwitch.get());
        Logger.recordOutput("Climb/RaisingArm", raisingArm);
        Logger.recordOutput("Climb/LoweringArm", loweringArm);
        setpoint = profile.calculate(0.02, setpoint, goal);
        double voltage = pid.calculate(inputs.absolutePosition, setpoint.position) +
                feedforward.calculate(setpoint.velocity, 0.0);
        // If upperSwitch is tripped
        if (!upperSwitch.get()) {
            // if (climbing) {
            // loweringArm = true;
            // }
            // raisingArm = false;
            if (voltage > 0) {
                voltage = 0;
            }
            // goal = new TrapezoidProfile.State(setpoint.position, 0);
        }
        // If lowerSwitch is tripped
        if (!lowerSwitch.get()) {
            // if (climbing) {
            // raisingArm = true;
            // }
            // loweringArm = false;
            if (voltage < 0) {
                voltage = 0;
            }
            io.setPosition(0);
            // goal = new TrapezoidProfile.State(setpoint.position, 0);
        }

        // if (raisingArm) {
        // io.runVoltage(ClimbConstants.RAISE_VOLTAGE);
        // return;
        // }
        // if (loweringArm) {
        // io.runVoltage(ClimbConstants.LOWER_VOLTAGE);
        // return;
        // }
        //io.runVoltage(voltage);
    }
}