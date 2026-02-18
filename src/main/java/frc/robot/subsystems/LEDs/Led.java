package frc.robot.subsystems.LEDs;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.LEDConfigs;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.LedConstants;

public class Led extends SubsystemBase{
    private LedIO io;
    private boolean enabled = true;
    private Color allianceColor = Color.kWhite;
    private final LedIOInputsAutoLogged inputs = new LedIOInputsAutoLogged();

    private boolean warning = false;

    public Led(LedIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        if (allianceColor.equals(Color.kWhite)) {
            updateAllianceColor();
        }
        io.updateInputs(inputs);
        if (enabled) {
            io.periodic();
        }
        Logger.processInputs("LEDs", inputs);
    }

    public void setPattern(LEDPattern pattern) {
        io.setPattern(pattern);
    }

    public void updateAllianceColor() {
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
                allianceColor = Color.kBlue;
            } else {
                allianceColor = Color.kRed;
            }
        }
    }

    public void autonomousInit() {
        CommandScheduler.getInstance().schedule(auton());
    }

    public void teleopInit() {
        updateAllianceColor();
        String gameData = DriverStation.getGameSpecificMessage();
        boolean wonAuton = false;
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
        } else {
            CommandScheduler.getInstance().schedule(
                setToPattern(LEDPattern.solid(Color.kWhite).blink(LedConstants.BLINK_FREQUENCY))
                .andThen(transitionPeriod())
                .andThen(activePeriod())
                .andThen(activePeriod())
                .andThen(activePeriod())
                .andThen(activePeriod())
                .andThen(endgame())
                );
        }
    }

    public Command wonAuton() {
        return setToPattern(LEDPattern.solid(Color.kGreen).blink(LedConstants.BLINK_FREQUENCY))
            .andThen(transitionPeriod())
            .andThen(inactivePeriod())
            .andThen(activePeriod())
            .andThen(inactivePeriod())
            .andThen(activePeriod())
            .andThen(endgame());
    }

    public Command lostAuton() {
        return setToPattern(LEDPattern.solid(Color.kRed).blink(LedConstants.BLINK_FREQUENCY))
            .andThen(new WaitCommand(1))
            .andThen(setToAllianceColor())
            .andThen(new WaitCommand(9))
            .andThen(activePeriod())
            .andThen(inactivePeriod())
            .andThen(activePeriod())
            .andThen(inactivePeriod())
            .andThen(endgame());
    }

    public Command transitionPeriod() {
        return new WaitCommand(1)
            .andThen(setToAllianceColor())
            .andThen(new WaitCommand(9 - LedConstants.WARN_TIME))
            .andThen(breatheActive())
            .andThen(warn());
    }

    public Command activePeriod() {
        return setToAllianceColor()
            .andThen(new WaitCommand(25 - LedConstants.WARN_TIME))
            .andThen(breatheActive())
            .andThen(warn());
    }

    public Command inactivePeriod() {
        return setToPattern(LEDPattern.solid(Color.kWhite))
            .andThen(new WaitCommand(25 - LedConstants.WARN_TIME))
            .andThen(breatheInactive())
            .andThen(warn());
    }

    public Command warn() {
        warning = true;
        return new WaitCommand(LedConstants.WARN_TIME)
            .andThen(new InstantCommand(() -> warning = false));
    }

    public Command setToAllianceColor() {
        return setToPattern(LEDPattern.solid(allianceColor));
    }

    public Command breatheActive() {
        return setToPattern(LEDPattern.solid(allianceColor).breathe(LedConstants.BREATHE_FREQUENCY));
    }

    public Command breatheInactive() {
        return setToPattern(LEDPattern.solid(Color.kWhite).breathe(LedConstants.BREATHE_FREQUENCY));
    }

    public Command auton() {
        return setToPattern(LEDPattern
            .gradient(GradientType.kContinuous, Color.kMaroon, Color.kWhite)
            .scrollAtRelativeSpeed(Percent.per(Second).of(50))
        );
    }

    public Command endgame() {
        return setToPattern(LEDPattern
            .gradient(GradientType.kContinuous, Color.kMaroon, Color.kWhite)
            .scrollAtRelativeSpeed(Percent.per(Second).of(50))
        );
    }

    public Command setToPattern(LEDPattern pattern) {
        return runOnce(
            () -> io.setPattern(pattern));
    }

    public Color getAllianceColor() {
        return allianceColor;
    }
}
