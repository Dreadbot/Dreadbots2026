package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.Constants.HoodConstants;

public class Hood extends SubsystemBase {
    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    public Hood(HoodIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);
    }

    public double getAngle() {
        return inputs.angle;
    }

    public double getRPM() {
        return inputs.RPM;
    }

    public Command setSpeed(double joystickValue) {
        return startEnd(
            () -> io.setVoltage(joystickValue * 6),
            () -> io.setVoltage(0)
        );
    }
}
