package frc.robot.autons;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Commands;

import static frc.robot.autons.TrajsAndLocs.Trajectories.*;

import frc.robot.autons.TrajsAndLocs.HPReefPair;
import frc.robot.autons.TrajsAndLocs.HPStation;
import frc.robot.autons.TrajsAndLocs.ReefLocs;
import frc.robot.autons.TrajsAndLocs.StartingLocs;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Superstructure;

public class WaltAutonFactory {
    private final AutoFactory m_autoFactory;
    private AutoRoutine m_routine;
    private final Superstructure m_superstructure;

    private final Debouncer m_debouncer = new Debouncer(0.5);

    public WaltAutonFactory(
        AutoFactory autoFactory, 
        Superstructure superstructure) {
        m_autoFactory = autoFactory;
        m_routine = m_autoFactory.newRoutine("auton");
        m_superstructure = superstructure;
    }

    public AutoRoutine generateAuton(

    ) {
        AutoTrajectory firstScoreTraj = m_routine.trajectory(StartToReefTrajs.get(new Pair<StartingLocs , ReefLocs>(StartingLocs.MID, ReefLocs.REEF_H)));
        AutoTrajectory firstHPTraj = m_routine.trajectory(ReefToHPTrajs.get(new Pair<ReefLocs, HPStation>(ReefLocs.REEF_H, HPStation.HP_RIGHT)));

        m_routine.active().onTrue(
            Commands.sequence(
                firstScoreTraj.resetOdometry(),
                firstScoreTraj.cmd()
            )
        );

        firstScoreTraj.done()
            .onTrue(
                Commands.sequence(
                    m_superstructure.autonEleToL2Req(),
                    m_superstructure.autonScoreReq(),
                    Commands.waitUntil(m_superstructure.isBotBeamBreamBrokey().negate()),
                    firstHPTraj.cmd()
                )
            );
        return m_routine;
    }
}