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
   
    public AutoCommands(Drive drive, Slapdown slapdown, Indexer indexer, Climb climb, Flywheel flywheel, AutoAim aim) {
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

        factory
            .bind("intake", slapdown.intakeCommand())
            .bind("stopIntake", slapdown.stopIntakeCommand())
            .bind("slapdown", slapdown.goToIntakeCommand().alongWith(slapdown.intakeCommand()))
            .bind("slapup", slapdown.goToHomeCommand().alongWith(slapdown.stopIntakeCommand()));
    }

    //number 1
    public AutoRoutine leftCenterCenterClimb() {
        AutoRoutine routine = factory.newRoutine("LeftCenterCenterClimb");
        AutoTrajectory leftCenterCenterClimb = routine.trajectory("centerCenterClimbOutline")
        
        routine.active().onTrue(
                Commands.sequence(
                    leftCenterCenterClimb.resetOdometry(),
                    leftCenterCenterClimb.cmd(),
                    drive.stopDrive(),
                    aim.shoot().withTimeout(5.0)
            )
        );

        return routine;
    }

    //number 2
    public AutoRoutine depotClimb() {
        AutoRoutine routine = factory.newRoutine("depotClimb");
        AutoTrajectory startIntake1 = routine.trajectory("depotClimbOutline", 0);
        AutoTrajectory stopIntakeAndStartFire1 = routine.trajectory("depotClimbOutline", 1);
        AutoTrajectory stopFire1 = routine.trajectory("depotClimbOutline", 2);
        //AutoTrajectory climb1 = routine.trajectory("depotClimbOutline", 3);
        
        routine.active().onTrue(Commands.sequence(
            factory.resetOdometry("depotClimbOutline"),
            startIntake1.cmd().alongWith(slapdown.goToIntakeCommand()).alongWith(slapdown.intakeCommand()),
            stopIntakeAndStartFire1.cmd().alongWith(slapdown.stopIntakeCommand().andThen(aim.shoot())),
            stopFire1.cmd().alongWith(Commands.runOnce(() -> aim.stopShooting()))//,
            //climb1.cmd().alongWith(climb.levelOneClimb())
        ));
        return routine;
    }


    public AutoRoutine leftCenter() {
        AutoRoutine routine = factory.newRoutine("LeftCenter");
        AutoTrajectory leftCenter = routine.trajectory("LCProtect");
        
        routine.active().onTrue(
            Commands.sequence(
                leftCenter.resetOdometry(),
                leftCenter.cmd(),
                drive.stopDrive(),
                aim.shoot().withTimeout(5.0)
            )
        );

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

    //number 4 
    public AutoRoutine centerOutpostClimb() {
        AutoRoutine routine = factory.newRoutine("centerOutpostClimb");
        AutoTrajectory shoot1 = routine.trajectory("segment1", 0);
        AutoTrajectory intake1 = routine.trajectory("segment2", 1);
        AutoTrajectory stopSlapdown1 = routine.trajectory("segment3", 2);
        AutoTrajectory shoot2 = routine.trajectory("segment4", 3);
        AutoTrajectory climb1 = routine.trajectory("segement5", 4);
            
        routine.active().onTrue(Commands.sequence(
            aim.shoot().andThen(shoot1.cmd()),
            intake1.cmd().alongWith(slapdown.goToIntakeCommand().andThen(slapdown.intakeCommand())),
            slapdown.stopIntakeCommand().alongWith(stopSlapdown1.cmd()),
            shoot2.cmd().alongWith(aim.shoot()),
            climb1.cmd()
            /* .andThen(climb.levelOneClimb()) */
        ));
            return routine;
        }

    //number 5 
    public AutoRoutine centerFireCenterFire() {
        AutoRoutine routine = factory.newRoutine("centerFireCenterFire");
        AutoTrajectory shoot1 = routine.trajectory("segment1", 0);
        AutoTrajectory intake1 = routine.trajectory("segment2", 1);
        AutoTrajectory stopIntake1 = routine.trajectory("segment3", 2);
        AutoTrajectory shoot2 = routine.trajectory("segment4", 3);
        AutoTrajectory intake2 = routine.trajectory("segment5", 4);
        AutoTrajectory stopIntake2 = routine.trajectory("segment6", 5);
        //doesn't move, here to show the final segement
        AutoTrajectory shoot3 = routine.trajectory("segment6", 6);

        routine.active().onTrue(Commands.sequence(
            aim.shoot().andThen(shoot1.cmd()),
            intake1.cmd().alongWith(slapdown.goToIntakeCommand().andThen(slapdown.intakeCommand())),
            stopIntake1.cmd().alongWith(slapdown.stopIntakeCommand()),
            aim.shoot().andThen(shoot2.cmd()),
            intake2.cmd().alongWith(slapdown.intakeCommand()),
            stopIntake2.cmd().alongWith(slapdown.stopIntakeCommand()),
            aim.shoot()
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
        AutoTrajectory climb = routine.trajectory("segment8", 7);

            return routine;
        }
}