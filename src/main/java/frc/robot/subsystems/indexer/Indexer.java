package frc.robot.subsystems.indexer;
// imports necessary libraries and files
import frc.robot.Constants.IndexerConstants;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    
    // sets up private variables
    private IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    private IndexerIOInputsAutoLogged kickerInputs = new IndexerIOInputsAutoLogged();
    private final PIDController pid = new PIDController(0.013, 0.0, 0);
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0.001);
    private IndexerIO io;
    private double kickerTargetRPM = 0;
    private double storedVoltage = IndexerConstants.KICKER_VOLTAGE;

    // gets io from IndexerIO.java
    public Indexer(IndexerIO io) {
        this.io = io;
    }


    // runs the intake command
    public Command startIndexer() {
        return runOnce(() -> io.runSpindexerVoltage(IndexerConstants.SPINDEXER_VOLTAGE));
    }

    public Command stopIndexer() {
        return runOnce(() -> io.runSpindexerVoltage(0.0));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs, kickerInputs);
        Logger.processInputs("Indexer", inputs);
        Logger.processInputs("IndexerKicker", kickerInputs);
        
        double currentRPM = kickerInputs.RPM;
        double output = feedforward.calculate(kickerTargetRPM);
        double pidOutput = pid.calculate(currentRPM, kickerTargetRPM);
        
        if (kickerTargetRPM > 0) {
            io.runKickerVoltage(pidOutput + output);
        } else {
            io.runKickerVoltage(0.0);
            pid.reset(); 
        }
    }

    public Command startKicker() {
        return runOnce(() -> kickerTargetRPM = IndexerConstants.KICKER_VOLTAGE);
    }

    public Command stopKicker() {
        return runOnce(() -> kickerTargetRPM = 0.0);
    }

    // The increase / decrease kicker speed/volts commands (intended for every click)
    public Command increaseVolts() {
        return runOnce(() -> io.runKickerVoltage(storedVoltage += 1));
    }

    public Command decreaseVolts() {
        return runOnce(() -> io.runKickerVoltage(storedVoltage -= 1));
    }
}
