package frc.robot.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.slapdown.Slapdown;

public class AutoCommands {
    public AutoFactory factory;
    public Drive drive;
    public Slapdown slapdown;
    public Indexer indexer;
   
    public AutoCommands(Drive drive, Slapdown slapdown, Indexer indexer, AutoFactory factory) {
        this.drive = drive;
        this.slapdown = slapdown;
        this.indexer = indexer;
        this.factory = new AutoFactory(
            drive::getPose,
            drive::setPose,
            drive::followTrajectory,
            true,
            drive,
            drive::logTrajectory
        );
    }

    //number 1
    public AutoRoutine centerCenterClimb() {
        AutoRoutine routine = factory.newRoutine("centerCenterClimb");
        AutoTrajectory segment1 = routine.trajectory("segment1", 0);
        AutoTrajectory segment2 = routine.trajectory("segment2" 1);
        AutoTrajectory segment3 = routine.trajectory("segment3" 2);
        AutoTrajectory segment4 = routine.trajectory("segment4" 3);
        AutoTrajectory segment5 = routine.trajectory("segment5" 4);
    }

    //number 4 NOT FINISHED
    public AutoRoutine centerDepotClimb() {
        AutoRoutine routine = factory.newRoutine("centerDepotClimb");
        AutoTrajectory segment1 = routine.trajectory("segment1", 0);
        AutoTrajectory segment2 = routine.trajectory("segment2", 1);
        AutoTrajectory segment3 = routine.trajectory("segment3", 2);
        AutoTrajectory segment4 = routine.trajectory("segment4", 3);
        routine.active().onTrue(Commands.sequence(
            segment1.cmd(),
            segment2.cmd().alongWith(slapdown.goToIntakeCommand().andThen(slapdown.intakeCommand())),
            segment3.cmd(),
            segment4.cmd()
        ));
        return routine;
    }

}