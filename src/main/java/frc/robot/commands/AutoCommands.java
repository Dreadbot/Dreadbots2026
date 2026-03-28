package frc.robot.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.slapdown.Slapdown;
import frc.robot.subsystems.AutoAim;
import frc.robot.subsystems.climb.Climb;

public class AutoCommands {
    public AutoFactory factory;
    public Drive drive;
    public Slapdown slapdown;
    public Hood hood;
    public Indexer indexer;
    public Climb climb;
    public AutoAim aim;
    public Flywheel flywheel;
   
    public AutoCommands(Drive drive, Slapdown slapdown, Hood hood, Indexer indexer, Climb climb, Flywheel flywheel, AutoAim aim) {
        this.drive = drive;
        this.slapdown = slapdown;
        this.indexer = indexer;
        this.flywheel = flywheel;
        this.hood = hood;
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

    public AutoRoutine leftDouble() {
        AutoRoutine routine = factory.newRoutine("leftDouble");
        AutoTrajectory LCOutside = routine.trajectory("LCOutside");
        AutoTrajectory LCInside = routine.trajectory("LCInside");

        routine.active().onTrue(
                Commands.sequence(
                    LCOutside.resetOdometry(),
                    Commands.deadline(LCOutside.cmd(), aim.trackTarget()),
                    drive.stopDrive(),
                    aim.shoot().alongWith(slapdown.agitateCommand()).withTimeout(5.0),
                    Commands.runOnce(() -> hood.setSetpoint(0.0)),
                    Commands.deadline(LCInside.cmd(), aim.trackTarget()),
                    drive.stopDrive(),
                    aim.shoot().alongWith(slapdown.agitateCommand())
            )
        );

        return routine;
    }

    public AutoRoutine rightDouble() {
        AutoRoutine routine = factory.newRoutine("rightDouble");
        AutoTrajectory RCOutside = routine.trajectory("RCOutside");
        AutoTrajectory RCInside = routine.trajectory("RCInside");

        routine.active().onTrue(
                Commands.sequence(
                    RCOutside.resetOdometry(),
                    Commands.deadline(RCOutside.cmd(), aim.trackTarget()),
                    drive.stopDrive(),
                    aim.shoot().alongWith(slapdown.agitateCommand()).withTimeout(5.0),
                    Commands.runOnce(() -> hood.setSetpoint(0.0)),
                    Commands.deadline(RCInside.cmd(), aim.trackTarget()),
                    drive.stopDrive(),
                    aim.shoot().alongWith(slapdown.agitateCommand())
            )
        );

        return routine;
    }

    public AutoRoutine rightDoubleSafe() {
        AutoRoutine routine = factory.newRoutine("rightDoubleSafe");
        AutoTrajectory RCOutside = routine.trajectory("RCOutsideSafe");
        AutoTrajectory RCInside = routine.trajectory("RCInsideSafe");

        routine.active().onTrue(
                Commands.sequence(
                    RCOutside.resetOdometry(),
                    Commands.deadline(RCOutside.cmd(), aim.trackTarget()),
                    drive.stopDrive(),
                    aim.shoot().alongWith(slapdown.agitateCommand()).withTimeout(5.0),
                    Commands.runOnce(() -> hood.setSetpoint(0.0)),
                    Commands.deadline(RCInside.cmd(), aim.trackTarget()),
                    drive.stopDrive(),
                    aim.shoot().alongWith(slapdown.agitateCommand())
            )
        );

        return routine;
    }

    public AutoRoutine rightDoubleSweep() {
        AutoRoutine routine = factory.newRoutine("rightDoubleSweep");
        AutoTrajectory RCOutside = routine.trajectory("RCOutsideSafe");
        AutoTrajectory RCInside = routine.trajectory("RCInsideSweepSafe");

        routine.active().onTrue(
                Commands.sequence(
                    RCOutside.resetOdometry(),
                    Commands.deadline(RCOutside.cmd(), aim.trackTarget()),
                    drive.stopDrive(),
                    aim.shoot().alongWith(slapdown.agitateCommand()).withTimeout(5.0),
                    Commands.runOnce(() -> hood.setSetpoint(0.0)),
                    Commands.deadline(RCInside.cmd(), aim.trackTarget()),
                    drive.stopDrive(),
                    aim.shoot().alongWith(slapdown.agitateCommand())
            )
        );

        return routine;
    }

    public AutoRoutine leftDoubleSafe() {
        AutoRoutine routine = factory.newRoutine("leftDoubleSafe");
        AutoTrajectory LCOutside = routine.trajectory("LCOutsideSafe");
        AutoTrajectory LCInside = routine.trajectory("LCInsideSafe");

        routine.active().onTrue(
                Commands.sequence(
                    LCOutside.resetOdometry(),
                    Commands.deadline(LCOutside.cmd(), aim.trackTarget()),
                    drive.stopDrive(),
                    aim.shoot().alongWith(slapdown.agitateCommand()).withTimeout(5.0),
                    Commands.runOnce(() -> hood.setSetpoint(0.0)),
                    Commands.deadline(LCInside.cmd(), aim.trackTarget()),
                    drive.stopDrive(),
                    aim.shoot().alongWith(slapdown.agitateCommand())
            )
        );

        return routine;
    }

    public AutoRoutine RCOutpost() {
        AutoRoutine routine = factory.newRoutine("CenterOutpost");
        AutoTrajectory RCOutpost = routine.trajectory("RCOutpost");

        routine.active().onTrue(
                Commands.sequence(
                    RCOutpost.resetOdometry(),
                    RCOutpost.cmd(),
                    drive.stopDrive(),
                    aim.shoot().alongWith(slapdown.agitateCommand())
            )
        );

        return routine;
    }

    public AutoRoutine wheelRadius() {
        AutoRoutine routine = factory.newRoutine("wheelRadius");
        AutoTrajectory wheelRadius = routine.trajectory("wheelRadius");

        routine.active().onTrue(
                Commands.sequence(
                    wheelRadius.resetOdometry(),
                    wheelRadius.cmd(),
                    drive.stopDrive()
            )
        );

        return routine;
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
    public AutoRoutine outpostClimb() {
        AutoRoutine routine = factory.newRoutine("OutpostClimb");
        AutoTrajectory trajectory = routine.trajectory("outpostClimbOutline");
        
        routine.active().onTrue(
                Commands.sequence(
                    trajectory.resetOdometry(),
                    trajectory.cmd()
            )
        );
        return routine;
    }



    // number 4
    public AutoRoutine rightOutpostClimbOutline() {
        AutoRoutine routine = factory.newRoutine("RightOutpostClimbOutline");
        AutoTrajectory trajectory = routine.trajectory("rightOutpostClimbOutline");
        
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