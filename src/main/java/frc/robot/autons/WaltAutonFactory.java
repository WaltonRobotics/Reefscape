package frc.robot.autons;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
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

    private StartingLocs m_startLoc;
    private ReefLocs m_firstScoreLoc;
    private HPStation m_firstHPLoc;

    private ReefLocs m_reiterateScoreLoc;
    private HPStation m_reiterateHPStationLoc;

    private final Debouncer m_debouncer = new Debouncer(0.5);

    public WaltAutonFactory(
        AutoFactory autoFactory, 
        Superstructure superstructure,
        StartingLocs startLoc,
        ReefLocs firstScoreLoc,
        HPStation firstHPLoc,
        ReefLocs reiterateScoreLoc,
        HPStation reiterateHPStationLoc
    ) {
        m_autoFactory = autoFactory;
        m_routine = m_autoFactory.newRoutine("auton");
        m_superstructure = superstructure;

        m_startLoc = startLoc;
        m_firstScoreLoc = firstScoreLoc;
        m_firstHPLoc = firstHPLoc;

        m_reiterateScoreLoc = reiterateScoreLoc;
        m_reiterateHPStationLoc = reiterateHPStationLoc;
    }

    private Command scoreCmd() {
        return Commands.sequence(
            m_superstructure.autonEleToL2Req(),
            m_superstructure.autonScoreReq(),
            Commands.waitUntil(m_superstructure.isBotBeamBreamBrokey().negate())
        );
    }

    public AutoRoutine generateAuton() {
        AutoTrajectory firstScoreTraj = m_routine.trajectory(StartToReefTrajs.get(new Pair<StartingLocs , ReefLocs>(m_startLoc, m_firstScoreLoc)));
        AutoTrajectory firstHPTraj = m_routine.trajectory(ReefToHPTrajs.get(new Pair<ReefLocs, HPStation>(m_firstScoreLoc, m_firstHPLoc)));

        AutoTrajectory reiterateScoreTraj = m_routine.trajectory(HPToReefTrajs.get(new Pair<HPStation, ReefLocs>(m_firstHPLoc, m_reiterateScoreLoc)));
        AutoTrajectory reiterateHPTraj = m_routine.trajectory(ReefToHPTrajs.get(new Pair<ReefLocs, HPStation>(m_reiterateScoreLoc, m_reiterateHPStationLoc)));

        m_routine.active().onTrue(
            Commands.sequence(
                firstScoreTraj.resetOdometry(),
                firstScoreTraj.cmd()
            )
        );

        firstScoreTraj.done()
            .onTrue(
                Commands.sequence(
                    scoreCmd(),
                    firstHPTraj.cmd()
                )
            );
            
        firstHPTraj.done()
            .onTrue(
                Commands.sequence(
                    Commands.waitUntil(m_superstructure.isBotBeamBreamBrokey()),
                    reiterateScoreTraj.cmd()
                )
            );

        reiterateScoreTraj.done()
            .onTrue(
                Commands.sequence(
                    scoreCmd(),
                    reiterateHPTraj.cmd()
                )
            );

        return m_routine;
    }
}