package frc.robot.subsystems.LEDs;

import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.LedConstants;

public class Led extends SubsystemBase{
    private LedIO io;
    private boolean enabled;
    private Color allianceColor = Color.kWhite;
    private final LedIOInputsAutoLogged inputs = new LedIOInputsAutoLogged();

    public Led(LedIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        if (allianceColor == Color.kWhite && DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
                allianceColor = Color.kBlue;
            } else {
                allianceColor = Color.kRed;
            }
        }
        io.updateInputs(inputs);
        if (enabled) {
            io.periodic();
        }
        Logger.processInputs("LEDs", inputs);
    }

    public void teleopInit() {
        String gameData = DriverStation.getGameSpecificMessage();
        boolean wonAuton = false;
        setToPattern(LEDPattern.solid(allianceColor));
        if(gameData.length() > 0) {
            switch (gameData.charAt(0)){
                case 'B' :
                    wonAuton = (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Blue));
                break;
                case 'R' :
                    wonAuton = (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red));
                break;
                default :
                break;
            }
            if (wonAuton) {
                CommandScheduler.getInstance().schedule(wonAuton());
            } else {
                CommandScheduler.getInstance().schedule(lostAuton());
            }
        }
    }

    public Command wonAuton() {
        return setToPattern(LEDPattern.solid(Color.kGreen).blink(Seconds.of(0.2)))
            .andThen(transitionPeriod())
            .andThen(inactivePeriod())
            .andThen(activePeriod())
            .andThen(inactivePeriod())
            .andThen(activePeriod())
            .andThen(setToAllianceColor());
    }

    public Command lostAuton() {
        return setToPattern(LEDPattern.solid(Color.kRed).blink(Seconds.of(0.2)))
            .andThen(transitionPeriod())
            .andThen(activePeriod())
            .andThen(inactivePeriod())
            .andThen(activePeriod())
            .andThen(inactivePeriod())
            .andThen(setToAllianceColor());
    }

    public Command transitionPeriod() {
        return setToAllianceColor()
            .andThen(new WaitCommand(10 - LedConstants.WARN_TIME))
            .andThen(breatheActive())
            .andThen(new WaitCommand(LedConstants.WARN_TIME));
    }

    public Command activePeriod() {
        return setToAllianceColor()
            .andThen(new WaitCommand(25 - LedConstants.WARN_TIME))
            .andThen(breatheActive())
            .andThen(new WaitCommand(LedConstants.WARN_TIME));
    }

    public Command inactivePeriod() {
        return setToPattern(LEDPattern.solid(Color.kWhite))
            .andThen(new WaitCommand(25 - LedConstants.WARN_TIME))
            .andThen(breatheInactive())
            .andThen(new WaitCommand(LedConstants.WARN_TIME));
    }

    public Command setToAllianceColor() {
        return setToPattern(LEDPattern.solid(allianceColor));
    }

    public Command breatheActive() {
        return setToPattern(LEDPattern.solid(allianceColor).breathe(Seconds.of(0.5)));
    }

    public Command breatheInactive() {
        return setToPattern(LEDPattern.solid(Color.kWhite).breathe(Seconds.of(0.5)));
    }

    public Command setToPattern(LEDPattern pattern) {
        return runOnce(
            () -> io.setPattern(pattern));
    }

    public Color getAllianceColor() {
        return allianceColor;
    }
}
