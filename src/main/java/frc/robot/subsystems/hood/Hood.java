package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import frc.robot.Constants.HoodConstants;

public class Hood extends SubsystemBase {
    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
    private final PIDController pid = new PIDController(HoodConstants.HOOD_KP, 0, 0);
    
    private double goalRotations = 0.0;
    private boolean calibrating = false;

    public Hood(HoodIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);
        
        if (calibrating) {
            if (inputs.lowerSwitch) {
                io.setPosition(0);
                goalRotations = 0.0;
                calibrating = false;
                io.setVoltage(0);
            }
            return;
        }

        double pidVoltage = pid.calculate(inputs.rotations, goalRotations);
        if (pidVoltage > 0 && inputs.rotations >= HoodConstants.MAX_ROTATIONS) {
            pidVoltage = 0;
        }
        pidVoltage = MathUtil.clamp(pidVoltage, -HoodConstants.MAX_VOLTAGE, HoodConstants.MAX_VOLTAGE);

        if (inputs.lowerSwitch) {
            io.setPosition(0);
            //goalRotations = 0;
            if (pidVoltage < 0) pidVoltage = 0;
        }
        io.setVoltage(pidVoltage);
    }

    public double getRotations() {
        return inputs.rotations;
    }

    public Command calibrate() {
        return runOnce(() -> {
            if (!inputs.lowerSwitch) {
                calibrating = true;
                io.setVoltage(-0.2);
            }
        });
    }

    public Command setRotations(double rotations) {
        return runOnce(() -> goalRotations = rotations);
    }

    public Command changeRotations(double rotations) {
        return runOnce(() -> goalRotations += rotations);
    }
}