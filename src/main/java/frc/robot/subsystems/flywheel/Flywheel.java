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
    private final PIDController pid = new PIDController(0.0002, 0, 0);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.09, 0.15, 5.35, 0.15);
    
    private double storedVoltage = FlywheelConstants.SHOOT_VOLTAGE;
    private double goalRPM = 0.0;

    public Flywheel(FlywheelIO io) {
        this.io = io;
        pid.setTolerance(FlywheelConstants.RPM_TOLERANCE);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Flywheel", inputs);

        double pidValue = pid.calculate(inputs.RPM, goalRPM);
        double feedforwardValue = feedforward.calculateWithVelocities(inputs.RPM, goalRPM);
        io.setVoltage(pidValue + (goalRPM / 525));
        // io.setRPM(goalRPM); // For sparkflex PID system
        Logger.recordOutput("Flywheel/GoalRPM", goalRPM);
        Logger.recordOutput("Flywheel/PIDValue", pidValue);
        Logger.recordOutput("Flywheel/FeedforwardValue", feedforwardValue);
        Logger.recordOutput("Flywheel/ActualRPM", inputs.RPM);
    }

    public double getRPM() {
        return inputs.RPM;
    }

    public boolean atRPM() {
        return pid.atSetpoint();
    }

    // These commands are for just starting and stopping at a set voltage
    public Command start() {
        return startEnd(
            () -> io.setVoltage(FlywheelConstants.SHOOT_VOLTAGE),
            () -> io.setVoltage(0.0)
        );
    }

    public Command stop() {
        return startEnd(
            () -> io.setVoltage(0.0),
            () -> {}
        );
    }

    // These commands work with the PID and feedforward to reach a set RPM
    public Command setRPM(double rpm) {
        return runOnce(() -> goalRPM = rpm);
    }

    public Command changeRPM(double rpm) {
        return runOnce(() -> goalRPM += rpm);
    }

    // The increase / decrease flywheel speed/volts commands (intended for every click)
    public Command increaseVolts() {
        return runOnce(() -> io.setVoltage(storedVoltage += 1));
    }

    public Command decreaseVolts() {
        return runOnce(() -> io.setVoltage(storedVoltage -= 1));
    }