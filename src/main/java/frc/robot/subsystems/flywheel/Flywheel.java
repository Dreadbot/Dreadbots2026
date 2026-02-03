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

    private final PIDController pid = new PIDController(0.01, 0, 0);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.09, 0.15, 5.35, 0.15);

    private double goalRPM = 0.0;

    public Flywheel(FlywheelIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Flywheel", inputs);

        double pidValue = pid.calculate(inputs.RPM, goalRPM);
        double feedforwardValue = feedforward.calculateWithVelocities(inputs.RPM, goalRPM);
        // io.setVoltage(pidValue + 0);
        // Logger.recordOutput("Flywheel/GoalRPM", goalRPM);
        // Logger.recordOutput("Flywheel/PIDValue", pidValue);
        // Logger.recordOutput("Flywheel/FeedforwardValue", feedforwardValue);
        // Logger.recordOutput("Flywheel/ActualRPM", inputs.RPM);
    }

    public void runAtVoltage(double volts) {
        io.setVoltage(volts);
    }

    public double getRPM() {
        return inputs.RPM;
    }

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

    public Command setSpeed(double rpm) {
        return runOnce(() -> goalRPM = rpm);
    }
}
