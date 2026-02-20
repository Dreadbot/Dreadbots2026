package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;

public class Climb extends SubsystemBase {
    // Auto logging output to something
    private ClimbIO io;
    private ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    // PID work?
    private final PIDController pid = new PIDController(0.013, 0.0, 0);
    private final TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(540, 540));
    private TrapezoidProfile.State goal = new TrapezoidProfile.State(0, 0);
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
    public final ArmFeedforward feedforward = new ArmFeedforward(0.00, 0.0, 0.023);
    private DigitalInput lowerSwitch = new DigitalInput(ClimbConstants.LOWER_LIMIT_SWITCH_ID);
    private DigitalInput upperSwitch = new DigitalInput(ClimbConstants.UPPER_LIMIT_SWITCH_ID);

    @AutoLogOutput
    // Setting up the boolean Varible, which is for right now isClimbed (Basic will
    // be updated later)
    public boolean isClimbed = false;
    public boolean raisingArm = false;
    public boolean loweringArm = false;

    public Climb(ClimbIO io) {
        this.io = io;
    }
    
    public Command motorForward() {
        return Commands.startEnd(
            () -> {
                if (upperSwitch.get()) raisingArm = true;
            },
            () -> raisingArm = false
        );
    }

    public Command motorBackward() {
        return Commands.startEnd(
            () -> {
                if (lowerSwitch.get()) loweringArm = true;
            },
            () -> loweringArm = false
        );
    }

    public Command raiseClimbArm() {
        return Commands.runOnce(() -> {
            if (upperSwitch.get()) raisingArm = true;
        });
    }

    public Command raiseRobotLevelOne() {
        return Commands.runOnce (() -> {
            if (lowerSwitch.get()) goal = new TrapezoidProfile.State(ClimbConstants.LEVEL_ONE_CLIMB_POSITION, 0);
        });
    }

    public Command levelOneClimb() {
        return Commands.sequence(
            raiseClimbArm(),
            Commands.waitUntil(() -> raisingArm == false),
            raiseRobotLevelOne()
        );
    }

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
        Logger.recordOutput("Climb/LowerSwitch", lowerSwitch.get());
        Logger.recordOutput("Climb/UpperSwitch", upperSwitch.get());
        Logger.recordOutput("Climb/RaisingArm", raisingArm);
        Logger.recordOutput("Climb/LoweringArm", loweringArm);
        setpoint = profile.calculate(0.02, setpoint, goal);

        // If upperSwitch is tripped
        if(!upperSwitch.get()) {
            raisingArm = false;
            io.setPosition(0);
            goal = new TrapezoidProfile.State(setpoint.position, 0);
        }
        // If lowerSwitch is tripped
        if(!lowerSwitch.get()) {
            loweringArm = false;
            goal = new TrapezoidProfile.State(setpoint.position, 0);
        }

        if (raisingArm) {
            io.runVoltage(ClimbConstants.RAISE_VOLTAGE);
            return;
        }
        if (loweringArm) {
            io.runVoltage(ClimbConstants.LOWER_VOLTAGE);
            return;
        }
        io.runVoltage(
            pid.calculate(inputs.absolutePosition, setpoint.position) +
            feedforward.calculate(inputs.absolutePosition + 90, setpoint.velocity)
            // use acutal position degrees to make sure that we always apply the correct
            // gravity feed forward.
        );
    }
}