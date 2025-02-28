package frc.robot.autons;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Commands;

import static frc.robot.autons.TrajsAndLocs.Trajectories.*;

import frc.robot.autons.TrajsAndLocs.ReefLocs;
import frc.robot.autons.TrajsAndLocs.StartingLocs;
import frc.robot.subsystems.Superstructure;

public class WaltAutonFactory {
    private final AutoFactory m_autoFactory;
    private final Superstructure m_superstructure;
    private AutoRoutine m_routine;

    public WaltAutonFactory(AutoFactory autoFactory, Superstructure superstructure) {
        m_autoFactory = autoFactory;
        m_superstructure = superstructure;
        m_routine = m_autoFactory.newRoutine("auton");
    }

    public AutoRoutine generateAuton(

    ) {
        AutoTrajectory firstScoreTraj = m_routine.trajectory(StartToReefTrajs.get(new Pair<StartingLocs , ReefLocs>(StartingLocs.MID, ReefLocs.REEF_H)));

        m_routine.active().onTrue(
            Commands.sequence(
                firstScoreTraj.resetOdometry(),
                firstScoreTraj.cmd()
            )
        );

        return m_routine;
    }
}