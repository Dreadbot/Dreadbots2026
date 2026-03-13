package frc.robot.subsystems.indexer;
// imports necessary libraries and files
import frc.robot.Constants.IndexerConstants;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    
    // sets up private variables
    private IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    private IndexerIOInputsAutoLogged kickerInputs = new IndexerIOInputsAutoLogged();
    private final PIDController pid = new PIDController(IndexerConstants.KICKER_KP, IndexerConstants.KICKER_KI, IndexerConstants.KICKER_KD);
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(IndexerConstants.KICKER_KS, IndexerConstants.KICKER_KV);
    private IndexerIO io;
    private double kickerTargetRPM = 0;
    private boolean isFeeding = false;

    // gets io from IndexerIO.java
    public Indexer(IndexerIO io) {
        this.io = io;
        SmartDashboard.putData("KickerPID", pid);
        SmartDashboard.putNumberArray("KickerFF", new double[] {IndexerConstants.KICKER_KS, IndexerConstants.KICKER_KV});
        SmartDashboard.putData("KickerFFUpdate", updateFF());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        io.updateInputs(kickerInputs);
        //Logger.processInputs("Indexer", inputs);
        Logger.processInputs("Kicker", kickerInputs);
        Logger.recordOutput("Kicker/TargetRPM", kickerTargetRPM);
        
        double currentRPM = kickerInputs.RPM;
        double output = feedforward.calculate(kickerTargetRPM);
        double pidOutput = pid.calculate(currentRPM, kickerTargetRPM);
        if (pidOutput < 0) {
            pidOutput = 0;
        }

        if (kickerTargetRPM > 0) {
            io.runKickerVoltage(pidOutput + output);
        } else {
            io.runKickerVoltage(0.0);
            pid.reset(); 
        }
    }

    public void startIndexing() {
        io.runSpindexerVoltage(IndexerConstants.SPINDEXER_VOLTAGE);
        io.runKickerVoltage(IndexerConstants.KICKER_VOLTAGE);
        isFeeding = true;
    }

    public void startReverseIndexing() {
        io.runSpindexerVoltage(IndexerConstants.SPINDEXER_VOLTAGE * -1);
        io.runKickerVoltage(IndexerConstants.KICKER_VOLTAGE * -1);
        isFeeding = false;
    }

    public void stopIndexing() {
        io.runSpindexerVoltage(0.0);
        io.runKickerVoltage(0.0);
        isFeeding = false;
    }

    public Command conditionalFeed(BooleanSupplier supplier) {
        return Commands.run(() -> {
            if (supplier.getAsBoolean() == isFeeding) {
                return;
            }
            if (isFeeding) {
                stopIndexing();
                return;
            }
            startIndexing();
        }, this);
    }

    public Command updateFF() {
        return Commands.runOnce(() -> {
            double[] nums = SmartDashboard.getNumberArray("KickerFF", new double[] {IndexerConstants.KICKER_KS, IndexerConstants.KICKER_KV});
            feedforward.setKs(nums[0]);
            feedforward.setKv(nums[1]);
        });
    }

    // The increase / decrease kicker speed/volts commands (intended for every click)
    // public Command increaseVolts() {
    //     return runOnce(() -> io.runKickerVoltage(storedVoltage += 1));
    // }

    // public Command decreaseVolts() {
    //     return runOnce(() -> io.runKickerVoltage(storedVoltage -= 1));
    // }
}
