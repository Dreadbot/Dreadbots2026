package frc.robot.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.drive.Drive;


public class AutoCommands {
    private AutoFactory factory;
    private Drive drive;


    public AutoCommands(Drive drive) {
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



    public Command WheelRadiCalibration() {
        return Commands.sequence(
            factory.resetOdometry("WheelRadiCalibration"),
            factory.trajectoryCmd("WheelRadiCalibration")
        );
    }

    public Command StraightLine(){
        return Commands.sequence(
            factory.resetOdometry("StraightLine"),
            factory.trajectoryCmd("StraightLine")
        );
    }

    public Command StraightLineBack(){
        return Commands.sequence(
            factory.resetOdometry("StraightLineBack"),
            factory.trajectoryCmd("StraightLineBack")
        );
    }
}