package frc.robot.subsystems.slapdown;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SlapdownConstants;
import frc.robot.subsystems.drive.DriveConstants;

public class Slapdown extends SubsystemBase {
    Twist2d positions = new Twist2d();
    ChassisSpeeds speeds = new ChassisSpeeds();

    private final SlapdownIOInputsAutoLogged inputs = new SlapdownIOInputsAutoLogged();
    private final SlapdownIO io;
    public final PIDController pid = new PIDController(SlapdownConstants.KP, SlapdownConstants.KI,
            SlapdownConstants.KD);
    public final ArmFeedforward feedforward = new ArmFeedforward(SlapdownConstants.KS, SlapdownConstants.KG,
            SlapdownConstants.KV);

    /*
    Constraints are split into two states, initial movement and final movement. Initial is faster to 
    counteract the slowness when extending the slapdown
    */
    private final TrapezoidProfile.Constraints finalProfile = new TrapezoidProfile.Constraints(
            SlapdownConstants.MAX_VELOCITY, SlapdownConstants.MAX_ACCELERATION);
    private final TrapezoidProfile.Constraints initialProfile = new TrapezoidProfile.Constraints(
            SlapdownConstants.MAX_VELOCITY + 100, SlapdownConstants.MAX_ACCELERATION); //placeholder
    private TrapezoidProfile.State goal = new TrapezoidProfile.State(0, 0);
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    public Slapdown(SlapdownIO io) {
        this.io = io;
        SmartDashboard.putData("SlapdownPID", pid);
    }

    public Command goToIntakeCommand() {
        return Commands.runOnce(
                () -> goal = new TrapezoidProfile.State(SlapdownConstants.INTAKE_ANGLE_DEGREES, 0));
    }

    public Command goToHomeCommand() {
        return Commands.runOnce(
                () -> goal = new TrapezoidProfile.State(SlapdownConstants.HOME_ANGLE_DEGREES, 0));
    }

    public Command outtakeCommand() {
        return Commands.runEnd(
            () -> {
                if (canRunIntake()) {
                    io.runIntakeVoltage(-SlapdownConstants.INTAKE_VOLTAGE);
                }
            },
            () -> io.runIntakeVoltage(0.0)
        );
    }

    public Command intakeCommand() {
        return Commands.runEnd(
            () -> {
                if (canRunIntake()) {
                    io.runIntakeVoltage(relativeIntakeSpeed());
                }
            },
            () -> io.runIntakeVoltage(0.0)
        );
    }

    public Command stopIntakeCommand() {
        return Commands.runOnce(
            () -> io.runIntakeVoltage(0.0)
        );
    }

    // Double check with test
    public Command agitateCommand() {
        return intakeCommand().withTimeout(0.25).andThen(
            Commands.repeatingSequence(
                outtakeCommand().withTimeout(0.075),
                intakeCommand().withTimeout(0.25),
                Commands.waitSeconds(0.1)
            // ).alongWith(Commands.repeatingSequence(
            //     setAngleDegrees(105),
            //     Commands.waitSeconds(0.5),
            //     goToIntakeCommand(),
            //     Commands.waitSeconds(0.5)
            // )
        )).finallyDo(interrupted -> goToIntakeCommand());
    }

    private boolean canRunIntake() {
        return inputs.absolutePosition > 105;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("SlapdownIntake", inputs);

        if (DriverStation.isDisabled()) {
            setpoint = new TrapezoidProfile.State(inputs.absolutePosition, 0);
            goal = setpoint;
        }

        TrapezoidProfile.Constraints constraints;

        /*
        checks every .2 millisecs to see which constraints the profile needs
        */
        //placeholder angle
        if (getAngle() < SlapdownConstants.MIN_ANGLE_RAD + Units.degreesToRadians(10)) {
            constraints = initialProfile;
        } else {
            constraints = finalProfile;
        }

        TrapezoidProfile profile = new TrapezoidProfile(constraints);
        setpoint = profile.calculate(0.02, setpoint, goal);
        double voltage = pid.calculate(inputs.absolutePosition, setpoint.position);
        if (inputs.absolutePosition < 20 && voltage > 0.6) {
            voltage += 3;
        }
        
        double ffvoltage = feedforward.calculate(Units.degreesToRadians(setpoint.position - 90), setpoint.velocity);

        Logger.recordOutput("Slapdown/SetpointPosition", setpoint.position);
        Logger.recordOutput("Slapdown/GoalPosition", goal.position);
        Logger.recordOutput("Slapdown/Voltage", voltage);
        Logger.recordOutput("Slapdown/FFVoltage", ffvoltage);
        io.runPivotVoltage(voltage);
    }

    public Command setAngleDegrees(double angle) {
        return runOnce(
                () -> goal = new TrapezoidProfile.State(angle, 0));
    }

    public double getAngle() {
        return inputs.absolutePosition;
    }

    public double getDotProduct() {
        double headingAngle = positions.dtheta;
        double xVelo = speeds.vxMetersPerSecond;
        double yVelo = speeds.vyMetersPerSecond;

        double dotProduct = Math.cos(headingAngle) * xVelo + Math.sin(headingAngle) * yVelo;
        return dotProduct;
    }

    public double relativeIntakeSpeed() {
        // i is the intake voltage
        // v is the dot product range
        double dotProduct = getDotProduct();
        double iMin = SlapdownConstants.INTAKE_VOLTAGE/2;
        double iMax = SlapdownConstants.INTAKE_VOLTAGE;
        double vMin = 0;
        double vMax = DriveConstants.maxSpeedMetersPerSec;;
        double i;

        if (dotProduct < vMin){
            i = iMin;
        } else if (dotProduct > vMax) {
            i = iMax;
        } else {
            i = ((dotProduct - vMin)/(vMax - vMin))*(iMax - iMin) + iMin;
        }
        return i;
    }
    }
