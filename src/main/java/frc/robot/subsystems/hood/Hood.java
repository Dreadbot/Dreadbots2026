package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import frc.robot.Constants.HoodConstants;

public class Hood extends SubsystemBase {
    // sets up private variables
    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
    private final PIDController pid = new PIDController(HoodConstants.HOOD_KP, HoodConstants.HOOD_KI, HoodConstants.HOOD_KD);
    
    // sets up goal rotations and calibrating boolean
    private double goalRotations = 0.0;
    private boolean calibrating = true;

    // gets io from HoodIO.java
    public Hood(HoodIO io) {
        this.io = io;
        pid.setTolerance(0.5);
    }

    /*
     * updates inputs and processes them, then checks if the hood is calibrating or at 0 rotations.
     * If it is, it runs the motor at a low voltage until the lower limit switch is triggered, at which point it sets the position to 0 and stops calibrating.
     * If it's not calibrating, it calculates the PID output to move towards the goal rotations, applying a feedforward voltage to overcome static friction.
     *  It also ensures that the hood does not move beyond its maximum rotations and that it does not apply negative voltage when the lower limit switch is triggered.
    */
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);
        
        if (calibrating || goalRotations == 0) {
            if (inputs.lowerSwitch) {
                io.setPosition(0);
                goalRotations = 0.0;
                calibrating = false;
                io.setVoltage(0);
                return;
            }
            io.setVoltage(-4);
            return;
        }

        // Calculate PID output and apply feedforward to overcome static friction
        double pidVoltage = pid.calculate(inputs.rotations, goalRotations);
        // Prevent moving beyond maximum rotations
        if (pidVoltage > 0 && inputs.rotations >= HoodConstants.MAX_ROTATIONS) {
            pidVoltage = 0;
        }
        // Add feedforward voltage to overcome static friction, only if the PID output is significant to avoid adding feedforward when the error is very small
        if (Math.abs(pidVoltage) > 0.05) {
            pidVoltage += Math.copySign(HoodConstants.HOOD_KS, pidVoltage);
        }
        // Clamp voltage to maximum limits
        pidVoltage = MathUtil.clamp(pidVoltage, -HoodConstants.MAX_VOLTAGE, HoodConstants.MAX_VOLTAGE);
        // Prevent applying negative voltage when the lower limit switch is triggered
        if (inputs.lowerSwitch) {
            io.setPosition(0);
            if (pidVoltage < 0) pidVoltage = 0;
        }
        // Set the motor voltage
        io.setVoltage(pidVoltage);
        // Log relevant information for debugging and analysis
        Logger.recordOutput("Hood/AtSetpoint", atSetpoint());
        Logger.recordOutput("Hood/Voltage", pidVoltage);
        Logger.recordOutput("Hood/Setpoint", goalRotations);
        Logger.recordOutput("Hood/LimitSwitch", inputs.lowerSwitch);
    }

    // Public methods to control the hood subsystem
    public double getRotations() {
        return inputs.rotations;
    }

    // Method to start the calibration process, which will run until the lower limit switch is triggered, setting the position to 0 and allowing the hood to be used for normal operation
    public void calibrate() {
        calibrating = true;
    }

    // Method to set the desired position of the hood in rotations, which will be used by the PID controller to move the hood towards that position
    public void setSetpoint(double rotations) {
        goalRotations = rotations;
    }

    // Method to check if the hood is at the desired position within the tolerance set in the PID controller, which can be used to determine if the hood has reached its target position
    public boolean atSetpoint() {
        return pid.atSetpoint();
    }

    // Method to change the desired position of the hood by a certain number of rotations, which can be used to incrementally adjust the hood's position based on user input or other factors
    public Command changeRotations(double rotations) {
        return runOnce(() -> goalRotations += rotations);
    }
}