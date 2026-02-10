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
    public void startIndexer() {
        io.runSpindexerVoltage(IndexerConstants.SPINDEXER_VOLTAGE);
        
    }

    public void stopIndexer() {
        io.runSpindexerVoltage(0.0);
    }

    public void startKicker() {
        io.runKickerVoltage(IndexerConstants.KICKER_VOLTAGE);
    }

    public void stopKicker() {
        io.runKickerVoltage(0.0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs, kickerInputs);
        Logger.processInputs("Indexer", inputs);
        Logger.processInputs("IndexerKicker", kickerInputs);
    }
}
