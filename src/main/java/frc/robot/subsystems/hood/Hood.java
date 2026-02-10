package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.Constants.HoodConstants;

public class Hood extends SubsystemBase {
    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
    private final PIDController pid = new PIDController(0.0002, 0, 0);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.09, 0.15, 5.35, 0.15);

    private DigitalInput lowerSwitch = new DigitalInput(HoodConstants.LOWER_LIMIT_SWITCH_ID);
    
    private double goalAngle = 0.0;

    public Hood(HoodIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        double pidVoltage = pid.calculate(inputs.angle, goalAngle);
        double feedforwardVoltage = feedforward.calculate(inputs.velocity);
        if (pidVoltage > 0 && inputs.angle >= HoodConstants.MAX_ANGLE) {
            pidVoltage = 0;
        }
        io.setVoltage(pidVoltage);

        if (lowerSwitch.get()) {
            io.setPosition(0);
        }
    }

    public double getAngle() {
        return inputs.angle;
    }

    public double getVelocity() {
        return inputs.velocity;
    }

    public Command calibrate() {
        return runOnce(() -> {
            while (!lowerSwitch.get()) {
                io.setVoltage(-0.1);
            }
            io.setVoltage(0);
            io.setPosition(0);
        });
    }

    public Command setAngle(double radians) {
        return runOnce(() -> goalAngle = radians);
    }
}
