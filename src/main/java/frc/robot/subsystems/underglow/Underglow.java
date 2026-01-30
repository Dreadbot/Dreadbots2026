package frc.robot.subsystems.underglow;

import static edu.wpi.first.units.Units.Seconds;

import java.util.concurrent.TimeUnit;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.UnderglowConstants;

public class Underglow extends SubsystemBase{
    private UnderglowIO io;
    private boolean enabled;
    private Color allianceColor = Color.kWhite;
    private final UnderglowIOInputsAutoLogged inputs = new UnderglowIOInputsAutoLogged();
    
    public Underglow(UnderglowIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        if (allianceColor == Color.kWhite && DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get().equals(Alliance.Blue)){
                allianceColor = Color.kBlue;
            } else {
                allianceColor = Color.kRed;
            }
        }
        io.updateInputs(inputs);
        if (enabled) {
            io.periodic();
        }
        Logger.processInputs("Underglow", inputs);
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
                setToPattern(LEDPattern.solid(Color.kGreen)).alongWith(
                    new WaitCommand(3).andThen(setToPattern(LEDPattern.solid(allianceColor))));
            } else {
                setToPattern(LEDPattern.solid(Color.kWhite));
            }
        }
    }

    public Command setToPattern(LEDPattern pattern) {
        return runOnce(
            () -> io.setPattern(pattern));
    }

    public Color getAllianceColor() {
        return allianceColor;
    }
}
