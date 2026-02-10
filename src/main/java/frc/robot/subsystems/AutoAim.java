package frc.robot.subsystems;

import org.ejml.simple.SimpleMatrix;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.turret.Turret;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoAim {
    private final InterpolatingMatrixTreeMap<Double, N3, N1> firingTable = new InterpolatingMatrixTreeMap<Double, N3, N1>();
    private final Hood hood;
    private final Flywheel flywheel;
    private final Turret turret;
    private final Indexer indexer;
    private final Drive drive;

    public AutoAim(Turret turret, Hood hood, Flywheel flywheel, Indexer indexer, Drive drive) {
        this.turret = turret;
        this.hood = hood;
        this.flywheel = flywheel;
        this.indexer = indexer;
        this.drive = drive;
        //              Distance (m)
        firingTable.put(0.0,    getMatrix(0.0, 0.0, 0.0));
        firingTable.put(10.0,   getMatrix(5.0, 10.0, 5.0));
    }

    public Command shoot() {
        return prepShot().andThen(Commands.runEnd(() -> shootSequence(), () -> stopShooting()));
    }

    public Command prepShot() {
        return Commands.runOnce(() -> startShootSequence()).andThen(Commands.waitUntil(flywheel::atRPM));
    }

    public void startShootSequence() {
        setSetpoints();
        indexer.startIndexer();
        indexer.startKicker();
    }

    public void shootSequence() {
        setSetpoints();
    }

    public void stopShooting() {
        flywheel.setRPM(0);
        indexer.stopIndexer();
        indexer.stopKicker();
    }

    public void setSetpoints() {
        Matrix<N3, N1> firingValues = getFiringTableValues(0.0);
        flywheel.setRPM(firingValues.get(0, 0));
        hood.setAngle(firingValues.get(1, 0));
    }   

    public Matrix<N3, N1> getFiringTableValues(double distance) {
        return firingTable.get(distance);
    }

    public static Matrix<N3, N1> getMatrix(double hoodSetpoint, double flywheelSetpoint, double flightTimeSeconds) {
        return new Matrix<>(new SimpleMatrix(3, 1, true, new double[]{hoodSetpoint, flywheelSetpoint, flightTimeSeconds}));
    }
}