package frc.robot.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import frc.robot.subsystems.drive.Drive;

public class AutoCommands {
    public AutoFactory factory;
    public Drive drive;
   
    public AutoCommands(Drive drive, AutoFactory factory) {
        this.drive = drive;
        this.factory = new AutoFactory(
            drive::getPose,
            drive::setPose,
            drive::followTrajectory,
            true,
            drive,
            drive::logTrajectory
        );
    }
}