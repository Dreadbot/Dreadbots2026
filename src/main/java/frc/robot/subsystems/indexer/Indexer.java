package frc.robot.subsystems.indexer;
// imports necessary libraries and files
import frc.robot.Constants.IndexerConstants;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Indexer extends SubsystemBase {
    
    // sets up private variables
    private IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    private IndexerIOInputsAutoLogged kickerInputs = new IndexerIOInputsAutoLogged();
    private final PIDController pid = new PIDController(IndexerConstants.KICKER_KP, IndexerConstants.KICKER_KI, IndexerConstants.KICKER_KD);
    //private final BangBangController bangbang = new BangBangController(250);
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(IndexerConstants.KICKER_KS, IndexerConstants.KICKER_KV);
    private IndexerIO io;
    private double kickerTargetRPM = 0;
    private boolean isFeeding = false;
    private CommandXboxController operator;

    // gets io from IndexerIO.java
    public Indexer(IndexerIO io, CommandXboxController operator) {
        this.io = io;
        this.operator = operator;
        SmartDashboard.putData("KickerPID", pid);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs, kickerInputs);
        //Logger.processInputs("Indexer", inputs);
        Logger.processInputs("Kicker", kickerInputs);
        Logger.recordOutput("Kicker/TargetRPM", kickerTargetRPM);
        
        double currentRPM = kickerInputs.RPM;
        double feedforwardOutput = feedforward.calculate(kickerTargetRPM);
        double pidOutput = pid.calculate(currentRPM, kickerTargetRPM);
        //double bangbangOutput = bangbang.calculate(currentRPM, kickerTargetRPM) * 12;
        
        if (pidOutput < 0) {
            pidOutput = 0;
        }
        double voltage = pidOutput + feedforwardOutput;
        Logger.recordOutput("Kicker/Voltage", voltage);
        if (kickerTargetRPM > 0) {
            io.runKickerVoltage(voltage);
        } else {
            io.runKickerVoltage(0.0);
            pid.reset(); 
        }
    }

    public void startIndexing() {
        io.runSpindexerVoltage(IndexerConstants.SPINDEXER_VOLTAGE);
        //io.runKickerVoltage(IndexerConstants.KICKER_VOLTAGE);
        kickerTargetRPM = IndexerConstants.KICKER_RPM;
        isFeeding = true;
        operator.setRumble(RumbleType.kBothRumble, 0.5);
    }

    public void startReverseIndexing() {
        io.runSpindexerVoltage(IndexerConstants.SPINDEXER_VOLTAGE * -1);
        kickerTargetRPM = -IndexerConstants.KICKER_RPM;
        //io.runKickerVoltage(IndexerConstants.KICKER_VOLTAGE * -1);
        isFeeding = false;
        operator.setRumble(RumbleType.kBothRumble, 0.0);
    }

    public void stopIndexing() {
        io.runSpindexerVoltage(0.0);
        //io.runKickerVoltage(0.0);
        kickerTargetRPM = 0.0;
        isFeeding = false;
        operator.setRumble(RumbleType.kBothRumble, 0.0);
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

    // The increase / decrease kicker speed/volts commands (intended for every click)
    // public Command increaseVolts() {
    //     return runOnce(() -> io.runKickerVoltage(storedVoltage += 1));
    // }

    // public Command decreaseVolts() {
    //     return runOnce(() -> io.runKickerVoltage(storedVoltage -= 1));
    // }
}
