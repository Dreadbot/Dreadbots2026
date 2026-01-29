package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb  extends SubsystemBase {
    
    private ClimbIO io;
    private ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
    
    @AutoLogOutput
    public boolean isClimbed = false; 
    public Climb(ClimbIO io) { 
        this.io = io;
}
    @Override
    public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);
   
    }
}
