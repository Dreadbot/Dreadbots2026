package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.FlywheelConstants;

public class Flywheel extends SubsystemBase {
  
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private final FlywheelIO io;

  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
  }

  public void runAtVoltage(double volts) {
    io.setVoltage(volts);
  }

  public double getRPM() {
    return inputs.RPM;
  }

  public Command start() {
    return startEnd(
        () -> io.setVoltage(FlywheelConstants.SHOOT_VOLTAGE),
        () -> io.setVoltage(0.0)
    );
  }

  public Command stop() {
    return startEnd(
        () -> io.setVoltage(0.0),
        () -> {}
    );
  }
}
