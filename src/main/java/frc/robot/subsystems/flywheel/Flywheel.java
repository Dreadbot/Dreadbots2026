package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import frc.robot.Constants.FlywheelConstants;

public class Flywheel extends SubsystemBase {
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
    private final FlywheelIO io;

    private double kP = 0.0015;
    private double kI = 0.0004;
    private double kD = 0.0;
    private double kS = 0.0;
    private double kV = 0.0016;
    private double kA = 0.0;
    private PIDController pid = new PIDController(kP, kI, kD);
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    private double increment = 0.1;
    private String target = "kP";
    private String system = "PID";

    private boolean stopping = false;

    public Flywheel(FlywheelIO io) {
        this.io = io;
        pid.setTolerance(FlywheelConstants.RPM_TOLERANCE);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Flywheel", inputs);

        double pidValue = pid.calculate(inputs.RPM);
        double feedforwardValue = feedforward.calculateWithVelocities(inputs.RPM, pid.getSetpoint());
        
        // io.setRPM(goalRPM); // For sparkflex PID system
        Logger.recordOutput("Flywheel/GoalRPM", pid.getSetpoint());
        Logger.recordOutput("Flywheel/atRPM", atRPM());
        // Logger.recordOutput("Flywheel/PIDValue", pidValue);
        // Logger.recordOutput("Flywheel/FeedforwardValue", feedforwardValue);
        Logger.recordOutput("Flywheel/ActualRPM", inputs.RPM);
        // Logger.recordOutput("Flywheel/System", system);
        // Logger.recordOutput("Flywheel/Target", target);
        Logger.recordOutput("Flywheel/TargetValue", switch (target) {
            case "kP" -> kP;
            case "kI" -> kI;
            case "kD" -> kD;
            case "kS" -> kS;
            case "kV" -> kV;
            case "kA" -> kA;
            default -> 0.0;
        });
        Logger.recordOutput("Flywheel/Increment", increment);

        if (!stopping) {
            io.setVoltage(pidValue + feedforwardValue);
        } else {
            io.setVoltage(0.0);
            return;
        }
    }

    public double getRPM() {
        return inputs.RPM;
    }

    public boolean atRPM() {
        return pid.atSetpoint();
    }

    public Command stop() {
        return runOnce(() -> stopping = true);
    }

    // These commands work with the PID and feedforward to reach a set RPM
    public void setRPM(double rpm) {
        stopping = rpm == 0;
        pid.setSetpoint(rpm);
    }

    public Command changeRPM(double rpm) {
        return runOnce(() -> {
            setRPM(pid.getSetpoint() + rpm);
        });
    }

    public Command changeOrderOfMagnitude(int change) {
        return runOnce(() -> increment *= Math.pow(10, change));
    }

    public Command changeNumber(double value) {
        return runOnce(() -> {
            switch (target) {
                case "kP":
                    kP += value * increment;
                    pid.setP(kP);
                    break;
                case "kI":
                    kI += value * increment;
                    pid.setI(kI);
                    break;
                case "kD":
                    kD += value * increment;
                    pid.setD(kD);
                    break;
                case "kS":
                    kS += value * increment;
                    feedforward = new SimpleMotorFeedforward(kS, kV, kA);
                    break;
                case "kV":
                    kV += value * increment;
                    feedforward = new SimpleMotorFeedforward(kS, kV, kA);
                    break;
                case "kA":
                    kA += value * increment;
                    feedforward = new SimpleMotorFeedforward(kS, kV, kA);
            }
        });
    }

    public Command changeTarget(String direction) {
        return runOnce(() -> {
            if (system == "PID") {
                switch (direction) {
                case "left":
                    target = "kP";
                    break;
                case "up":
                    target = "kI";
                    break;
                case "right":
                    target = "kD";
                    break;
                }
            } else {
                switch (direction) {
                    case "left":
                        target = "kS";
                        break;
                    case "up":
                        target = "kV";
                        break;
                    case "right":
                        target = "kA";
                        break;
                }
            }
        });
    }

    public Command changeSystem() {
        return runOnce(() -> {
            if (system == "PID") {
                system = "Feedforward";
            } else {
                system = "PID";
            }
        });
    }
}