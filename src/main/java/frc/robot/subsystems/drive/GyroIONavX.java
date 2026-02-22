// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.drive.DriveConstants.*;

import com.studica.frc.Navx;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.Queue;

/** IO implementation for NavX. */
public class GyroIONavX implements GyroIO {
  private final Navx navX = new Navx(30, (int) odometryFrequency);
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  public GyroIONavX() {
    yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(() -> navX.getYaw().in(Degrees));
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.yawPosition = Rotation2d.fromDegrees(-navX.getYaw().in(Degrees));
    inputs.yawVelocityRadPerSec = -navX.getAngularVel()[2].in(RadiansPerSecond);
    inputs.connected = navX.getYaw().in(Degrees) != 360.0;
    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(-value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}