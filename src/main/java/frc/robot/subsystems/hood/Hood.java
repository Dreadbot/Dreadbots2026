package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.Constants.HoodConstants;

public class Hood extends SubsystemBase {
    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
    private final PIDController pid = new PIDController(0.15, 0, 0);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.09, 0.15, 5.35, 0.15);

    private DigitalInput lowerSwitch = new DigitalInput(HoodConstants.LOWER_LIMIT_SWITCH_ID);
    
    private double goalAngle = 0.0;
    private boolean calibrating = false;

    public Hood(HoodIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);
        
        if (calibrating) {
            if (!lowerSwitch.get()) {
                io.setPosition(0);
                goalAngle = 0.0;
                calibrating = false;
            }
            return;
        }

        double pidVoltage = pid.calculate(inputs.angle, goalAngle);
        double feedforwardVoltage = feedforward.calculate(inputs.RPM);
        if (pidVoltage > 0 && inputs.angle >= HoodConstants.MAX_ANGLE) {
            pidVoltage = 0;
        }
        if (!lowerSwitch.get()) {
            io.setPosition(0);
            goalAngle = 0;
            if (pidVoltage < 0) pidVoltage = 0;
        }
        io.setVoltage(pidVoltage);

        Logger.recordOutput("Hood/lowerSwitch", lowerSwitch.get());
    }

    public double getAngle() {
        return inputs.angle;
    }

    public double getRPM() {
        return inputs.RPM;
    }

    public double getPosition() {
        return inputs.position;
    }

    public Command calibrate() {
        return runOnce(() -> {
            if (lowerSwitch.get()) {
                calibrating = true;
                io.setVoltage(-0.2);
            }
        });
    }

    public Command setAngle(double radians) {
        return runOnce(() -> goalAngle = radians);
    }

    public Command changeAngle(double radians) {
        return runOnce(() -> goalAngle += radians);
    }

    public Command raiseHood() { 
        return runOnce(() -> {
            double position = getPosition() + Units.degreesToRadians(10); 

            if (position > HoodConstants.MAX_ANGLE) {
                position = HoodConstants.MAX_ANGLE;
            } 
            
            io.setPosition(position); 
        }); 
    }
    public Command lowerHood() { 
        return runOnce(() -> {
            double position = getPosition() - Units.degreesToRadians(10); 

            if (position < HoodConstants.MIN_ANGLE) {
                position = HoodConstants.MIN_ANGLE;
            } 
            
            io.setPosition(position); 
        }); 
    }
}
