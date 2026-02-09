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



    public Command StraightLine(){
        return Commands.sequence(
            factory.resetOdometry("StraightLine"),
            factory.trajectoryCmd("StraightLine")
        );
    }

    public Command LeftTrench(){
        return Commands.sequence(
            factory.resetOdometry("LeftTrench"),
            factory.trajectoryCmd("LeftTrench")
        );
    }
    public Command CenterRisk(){
        return Commands.sequence(
            factory.resetOdometry("CenterRisk"),
            factory.trajectoryCmd("CenterRisk")
        );
    }
    public Command FullCenter(){
        return Commands.sequence(
            factory.resetOdometry("FullCenter"),
            factory.trajectoryCmd("FullCenter")
        );
    }
    public Command DoubleRight(){
        return Commands.sequence(
            factory.resetOdometry("DoubleRight"),
            factory.trajectoryCmd("DoubleRight", 0)
                .andThen(drive.stopDrive())
                .andThen(Commands.waitSeconds(5)),
            factory.trajectoryCmd("DoubleRight", 1)
        );
    }
}