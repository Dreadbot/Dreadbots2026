package frc.robot.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.slapdown.Slapdown;
import frc.robot.subsystems.AutoAim;
import frc.robot.subsystems.climb.Climb;

public class AutoCommands {
    public AutoFactory factory;
    public Drive drive;
    public Slapdown slapdown;
    public Indexer indexer;
    public Climb climb;
    public AutoAim aim;
    public Flywheel flywheel;
   
    public AutoCommands(Drive drive, Slapdown slapdown, Indexer indexer, AutoFactory factory, Climb climb, Flywheel flywheel, AutoAim aim) {
        this.drive = drive;
        this.slapdown = slapdown;
        this.indexer = indexer;
        this.flywheel = flywheel;
        this.factory = new AutoFactory(
            drive::getPose,
            drive::setPose,
            drive::followTrajectory,
            true,
            drive,
            drive::logTrajectory
        );
        this.climb = climb;
        this.aim = aim;
    }

    //number 1
    public AutoRoutine centerCenterClimb() {
        AutoRoutine routine = factory.newRoutine("centerCenterClimb");
        AutoTrajectory startIntake1 = routine.trajectory("segment1", 0);
        AutoTrajectory stopIntake1 = routine.trajectory("segment2", 1);
        AutoTrajectory fire1 = routine.trajectory("segment3", 2);
        AutoTrajectory startIntake2 = routine.trajectory("segment4", 3);
        AutoTrajectory stopIntake2 = routine.trajectory("segment5", 4);
        AutoTrajectory startFire2 = routine.trajectory("segment6", 5);
        AutoTrajectory stopFire2 = routine.trajectory("segment7", 6);
        //AutoTrajectory climb1 = routine.trajectory("segment8", 7);

        routine.active().onTrue(Commands.sequence(
            startIntake1.cmd().alongWith(slapdown.goToIntakeCommand().andThen(slapdown.intakeCommand())),
            stopIntake1.cmd().alongWith(slapdown.stopIntakeCommand()),
            fire1.cmd().alongWith(aim.shoot()),
            startIntake2.cmd().alongWith(slapdown.intakeCommand()),
            stopIntake2.cmd().alongWith(slapdown.stopIntakeCommand()),
            startFire2.cmd().alongWith(aim.shoot()),
            stopFire2.cmd().alongWith(aim.shoot())//,
            //climb1.cmd().alongWith(climb.levelOneClimb())
        ));
        return routine;
    }

    //number 2
    public AutoRoutine depotClimb() {
        AutoRoutine routine = factory.newRoutine("depotClimb");
        AutoTrajectory startIntake1 = routine.trajectory("segment1", 0);
        AutoTrajectory stopIntakeAndStartFire1 = routine.trajectory("segment2", 1);
        AutoTrajectory stopFire1 = routine.trajectory("segment3", 2);
        AutoTrajectory climb1 = routine.trajectory("segment4", 3);
        
        routine.active().onTrue(Commands.sequence(
            startIntake1.cmd().alongWith(slapdown.intakeCommand()),
            stopIntakeAndStartFire1.cmd().alongWith(slapdown.stopIntakeCommand().andThen(aim.shoot())),
            stopFire1.cmd().alongWith(aim.stopShooting())//,
            //climb1.cmd().alongWith(climb.levelOneClimb())
        ));
        return routine;
    }

    //number 3 
    public AutoRoutine centerFireClimb() {
        AutoRoutine routine = factory.newRoutine("centerFireClimb");
        AutoTrajectory shoot1 = routine.trajectory("segment1", 0);
        AutoTrajectory depot1 = routine.trajectory("segment2", 1);
        AutoTrajectory climb1 = routine.trajectory("segment3", 2);

        routine.active().onTrue(Commands.sequence(
            shoot1.cmd().alongWith(aim.shoot()),
            depot1.cmd().alongWith(slapdown.goToIntakeCommand().andThen(slapdown.intakeCommand())),
            slapdown.goToHomeCommand().andThen(slapdown.stopIntakeCommand().andThen(climb1.cmd()
            /*.alongWith(climb.levelOneClimb())*/
            ))
        ));
        return routine;
    }

    //number 4 NOT FINISHED
    public AutoRoutine centerOutpostClimb() {
        AutoRoutine routine = factory.newRoutine("centerOutpostClimb");
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
        
    //number 6
    public AutoRoutine RightDepotOutpostClimbOutline() {
        AutoRoutine routine = factory.newRoutine("RightDepotOutpostClimbOutline");
        AutoTrajectory startIntake1 = routine.trajectory("segment2", 0);
        AutoTrajectory stopIntake1 = routine.trajectory("segment4", 1);
        AutoTrajectory startFire = routine.trajectory("segment7", 2);
        AutoTrajectory stopFire = routine.trajectory("segment8", 3);
        //AutoTrajectory climb = routine.trajectory("segment8", 7);

        routine.active().onTrue(Commands.sequence(
            startIntake1.cmd().alongWith(slapdown.goToIntakeCommand().andThen(slapdown.intakeCommand())),
            stopIntake1.cmd().alongWith(slapdown.stopIntakeCommand()),
            startFire.cmd().alongWith(aim.shoot()),
            stopFire.cmd().alongWith(aim.shoot())//,
            //climb.cmd().alongWith(climb.levelOneClimb())
        ));
            return routine;
        }
}