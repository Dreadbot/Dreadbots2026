package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIO.FlywheelIOInputs inputs = new FlywheelIO.FlywheelIOInputs();

  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public void runAtVoltage(double volts) {
    io.setVoltage(volts);
  }

  public double getRPM() {
    return inputs.velocityRPM;
  }

  public Command start() {
    return startEnd(
        () -> io.setVoltage(6.0), // Example voltage
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
