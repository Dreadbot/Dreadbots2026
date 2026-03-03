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
        AutoTrajectory leftCenterCenterClimb = routine.trajectory("centerCenterClimbOutline");
        
        routine.active().onTrue(
                Commands.sequence(
                    leftCenterCenterClimb.resetOdometry(),
                    leftCenterCenterClimb.cmd()
            )
        );

        return routine;
    }

    //number 2
    public AutoRoutine depotClimb() {
        AutoRoutine routine = factory.newRoutine("DepotClimb");
        AutoTrajectory depotClimb = routine.trajectory("depotClimbOutline");
        
        routine.active().onTrue(
                Commands.sequence(
                    depotClimb.resetOdometry(),
                    depotClimb.cmd()
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
        



    // number 4
    public AutoRoutine RightDepotOutpostClimbOutline() {
        AutoRoutine routine = factory.newRoutine("RightDepotOutpostClimbOutline");
        AutoTrajectory trajectory = routine.trajectory("RightDepotOutpostClimbOutline");
        
        routine.active().onTrue(
        Commands.sequence(
            trajectory.resetOdometry(),
            trajectory.cmd()
        )
    );

        //Start Shooting/Autoaiming and shooting
        trajectory.atTime("shoot").onTrue(aim.shoot());

        //trajectory.atTime("climb").onTrue(climb.testRaise());

        return routine;
    }

    // number 5
    public AutoRoutine centerFireCenterFireOutline(){
        AutoRoutine routine = factory.newRoutine("centerFireCenterFireOutline");
        AutoTrajectory trajectory = routine.trajectory("centerFireCenterFireOutline");

        routine.active().onTrue(
            Commands.sequence(
                trajectory.resetOdometry(),
                trajectory.cmd()
            )
        );

        trajectory.atTime("shoot").onTrue(aim.shoot().withTimeout(3));

        return routine;
    }
}