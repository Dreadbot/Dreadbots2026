package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIO.HoodIOInputs inputs = new HoodIO.HoodIOInputs();

  public Hood(HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  public double getAngleDegrees() {
    return inputs.angleDeg;
  }

  public double getVelocityDegPerSec() {
    return inputs.velocityDegPerSec;
  }
}
