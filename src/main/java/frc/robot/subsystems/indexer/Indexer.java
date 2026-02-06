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
    private IndexerIO io;
    private boolean isIntaking = false;
    private boolean hasGamepiece = false;

    // gets io from IndexerIO.java
    public Indexer(IndexerIO io) {
        this.io = io;
    }



    // runs the intake command
    public Command intake() {
        return(
            Commands.startEnd(
            () -> io.runVoltage(IndexerConstants.INTAKE_VOLTAGE),
            () -> { io.runVoltage(0.0); isIntaking = false; }
            )
        );
    }

    public Command outtake() {
        return(
            Commands.startEnd(
            () -> io.runVoltage(IndexerConstants.OUTAKE_VOLTAGE),
            () -> { io.runVoltage(0.0); }
            )
        );
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
    }

}
