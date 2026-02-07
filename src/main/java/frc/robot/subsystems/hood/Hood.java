package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class Hood extends SubsystemBase {
    private final HoodIO io;
    private final HoodIO.HoodIOInputs inputs = new HoodIO.HoodIOInputs();
    private final PIDController pid = new PIDController(0.0002, 0, 0);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.09, 0.15, 5.35, 0.15);

    private double goalAngle = 0.0;

    public Hood(HoodIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        double pidValue = pid.calculate(inputs.angle, goalAngle);
        double feedforwardValue = feedforward.calculate(inputs.velocity);
        io.setVoltage(pidValue/* + (goalAngle / 1)*/);
    }

    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    public double getAngle() {
        return inputs.angle;
    }

    public double getVelocity() {
        return inputs.velocity;
    }

    public Command setAngle(double angle) {
        return runOnce(() -> goalAngle = angle);
    }
}
