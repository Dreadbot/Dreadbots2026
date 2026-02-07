package frc.robot.subsystems.hookprep;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hookprep extends SubsystemBase {
    private HookprepIO io;
    private HookprepIO.HookprepIOInputs inputs = new HookprepIO.HookprepIOInputs();

    public Hookprep(HookprepIO io) {
        this.io = io;
    }

    // maybe needed?
    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public Command readyHook(double joystickPosition) {
        double desiredPosition = joystickPosition + 1.0;
        int pulseWidth = (int)desiredPosition * 2000 + 500;
        
        return(
            Commands.startEnd(
                () -> {
                    io.runPulse(pulseWidth);
                },
                () -> {
                    io.runPulse(1500);
                }
            )
        );
    }
}
