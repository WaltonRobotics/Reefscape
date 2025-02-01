package frc.robot.autons;

import java.util.ArrayList;

import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autons.TrajsAndLocs.HpStation;
import frc.robot.autons.TrajsAndLocs.ReefHpPair;
import frc.robot.autons.TrajsAndLocs.FirstScoringLocs;
import frc.robot.autons.TrajsAndLocs.HpReefPair;
import frc.robot.autons.TrajsAndLocs.ReefLocation;
import frc.robot.autons.TrajsAndLocs.Trajectories;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Elevator.EleHeight;

public class WaltAutonFactory {
    private final AutoFactory m_autoFactory;
    private final SequentialCommandGroup m_fullCmdSequence;
    private final AutoRoutine m_routine;

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    public WaltAutonFactory(AutoFactory autoFactory, Swerve swerve, Elevator ele) {
        m_autoFactory = autoFactory;
        m_routine = m_autoFactory.newRoutine("auton");
        m_fullCmdSequence = new SequentialCommandGroup(/* TODO: resetOdometry */);
    }

    class AutonCycle {
        public final ReefLocation reefLoc;
        public final EleHeight height;
        public final HpStation hpStation;
        public final ReefHpPair reefHpPair;
        public final HpReefPair hpReefPair;

        public AutonCycle (ReefLocation _reefLoc, EleHeight _height, HpStation _hpStation) {
            reefLoc = _reefLoc;
            height = _height;
            hpStation = _hpStation;
            reefHpPair = new ReefHpPair(reefLoc, hpStation);
            hpReefPair = new HpReefPair(hpStation, reefLoc);

            if (!Trajectories.ReefToHpMap.containsKey(reefHpPair)) {
                // crash code
            }

            if (!Trajectories.ReefToHpMap.containsKey(hpReefPair)) {
                // crash code
            }
        }
    };

    // ArrayList<AutonCycle> cycles = 
    //     new AutonCycle(ScoringLoc.REEF_A, EleHeight.L4, HpStation.CS_LEFT),
    //     new AutonCycle(ScoringLoc.REEF_A, EleHeight.L4, HpStation.CS_LEFT),
    //     new AutonCycle(ScoringLoc.REEF_A, EleHeight.L4, HpStation.CS_LEFT),
    //     new AutonCycle(ScoringLoc.REEF_A, EleHeight.L4, HpStation.CS_LEFT),
    //     new AutonCycle(ScoringLoc.REEF_A, EleHeight.L4, HpStation.CS_LEFT)
    // );

    public Command generateAuton(Swerve drivetrain, Elevator ele, FirstScoringLocs firstScoreLoc, EleHeight firstHeight, ArrayList<AutonCycle> cycles) {
        // iterate AutonCycles and validate all Hp->Score/Score->Hp pairs
        // publish ElasticAlert if invalid pair is asked for
        AutoTrajectory firstScoreTraj = m_routine.trajectory(firstScoreLoc.m_startAndTraj.getSecond());
        AutoTrajectory firstLoadTraj = m_routine.trajectory("cs_2_right");

        m_routine.active().onTrue(
            Commands.sequence(
                firstScoreTraj.resetOdometry(),
                firstScoreTraj.cmd()
            )
        );

        firstScoreTraj.atTime("ElevUp").onTrue(ele.toPosition(firstHeight)); // pre-scoreReq to superstructure
        firstLoadTraj.atTime("Intake").onTrue(Commands.print("RunTheIntakePleeeaseee")); // autoIntakeReq to superstructure
        firstScoreTraj.done().onTrue(
            Commands.sequence(
                Commands.parallel(
                    drivetrain.applyRequest(() -> brake)
                    // superstructure.autoScore() // should send autoScoreRequest to state machine
                ),
                firstLoadTraj.cmd()
            )
        );
        // ^ ends at HP station

        // list of trajectory start commands
        ArrayList<AutoTrajectory> trajList = new ArrayList<>();
        for (int cycleIdx = 0; cycleIdx < cycles.size(); cycleIdx++ ) {
            // starts at HP station from last cycle
            AutonCycle cycle = cycles.get(cycleIdx);

            var hpToReefTraj = m_routine.trajectory(Trajectories.HpToReefMap.get(cycle.hpReefPair));
            var reefToHpTraj = m_routine.trajectory(Trajectories.ReefToHpMap.get(cycle.reefHpPair));

            trajList.add(reefToHpTraj);
            // attach this cycles first trajectory to the end of the last cycles last trajectory.
            if (cycles.size() > 1 && cycleIdx > 0) {
                var lastCycleDone = trajList.get(cycleIdx - 1).done(); 
                var gotACoral = new Trigger(() -> false); // from superstructure

                // once last cycles traj done, and got coral, begin moving to reef
                lastCycleDone.and(gotACoral).onTrue(hpToReefTraj.cmd());
            }
            // lift elevator early
            hpToReefTraj.atTime("ElevUp").onTrue(ele.toPosition(cycle.height));
            
            // robot is now at reef and finished scoring
            hpToReefTraj.done().onTrue(
                Commands.sequence(
                    Commands.parallel(
                        drivetrain.applyRequest(() -> brake)
                        // superstructure.autoScore() // should send autoScoreRequest to state machine
                    ),
                    // go back to HP
                    reefToHpTraj.cmd()
                )
            );
        }

        // start first cycle
        firstLoadTraj.done().onTrue(
            trajList.get(0).cmd()
        );

        return m_routine.cmd();
    }

    public AutoRoutine getAuton() {
        m_routine.active().onTrue(m_fullCmdSequence);

        return m_routine;
    }
}
