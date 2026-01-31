package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb  extends SubsystemBase {
//Auto logging output to something
    private ClimbIO io;
    private ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
    
    
    @AutoLogOutput
    // Setting up the boolean Varible, which is for right now isClimbed (Basic will be updated later)
    public boolean isClimbed = false; 
    public Climb(ClimbIO io) { 
        this.io = io;
    }

        public Command doClimbSequence() {
            return Commands.sequence(
                        Commands.startEnd(
                            () -> io.runVoltage(ClimbConstants.INTAKE_VOLTAGE),
                            () -> io.runVoltage(0.0)
                        )
            );
        }

        public Command unClimbSequence() {
                return Commands.sequence(
                        Commands.startEnd(
                            () -> io.runVoltage(ClimbConstants.OUTAKE_VOLTAGE),
                            () -> io.runVoltage(0.0)
                            )
                );
        }


    // Updates the inputs of ClimbIO perodic.
    // ClimbIO takes the inputs and outputs of Climb from the contorller
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climb", inputs);
    }
}
