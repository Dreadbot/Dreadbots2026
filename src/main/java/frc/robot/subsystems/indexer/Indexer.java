package frc.robot.subsystems.indexer;
// imports necessary libraries and files
import frc.robot.Constants.IndexerConstants;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    
    // sets up private variables
    private IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    private IndexerIOInputsAutoLogged kickerInputs = new IndexerIOInputsAutoLogged();
    private IndexerIO io;

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

    public Command startKicker() {
        return runOnce(() -> io.runKickerVoltage(IndexerConstants.KICKER_VOLTAGE));
    }

    public Command stopKicker() {
        return runOnce(() -> io.runKickerVoltage(0.0));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs, kickerInputs);
        Logger.processInputs("Indexer", inputs);
        Logger.processInputs("IndexerKicker", kickerInputs);
    }
}
