package frc.robot.subsystems.indexer;


import org.littletonrobotics.junction.Logger;


import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Indexer extends SubsystemBase {
   
    private IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    private IndexerIO io;
    private boolean isIntaking = false;
    private boolean hasGamepiece = false;


    public Indexer(IndexerIO io) {
        this.io = io;
    }


    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
    }


    // public Command intake() {
    //     return startEnd(
    //         () -> io.runVoltage(IndexerConstants.INTAKE_VOLTAGE),
    //         () -> { io.runVoltage(0.0); isIntaking = false; }
    //     );
    // }
}
