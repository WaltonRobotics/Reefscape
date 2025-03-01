package frc.robot.autons;

import java.util.ArrayList;

import javax.sound.sampled.SourceDataLine;

import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autons.TrajsAndLocs.HPReefPair;
import frc.robot.autons.TrajsAndLocs.HPStation;
import frc.robot.autons.TrajsAndLocs.ReefHPPair;
import frc.robot.autons.TrajsAndLocs.ReefLocs;
import frc.robot.autons.TrajsAndLocs.StartingLocs;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Swerve;
import frc.util.Elastic;
import frc.robot.subsystems.Elevator.EleHeight;
import frc.robot.autons.TrajsAndLocs.Trajectories;

public class WaltAutonFactory {
    private final AutoFactory m_autonFactory;
    private final AutoRoutine m_routine;

    private final SwerveRequest.SwerveDriveBrake m_brake = new SwerveRequest.SwerveDriveBrake();
    
    public WaltAutonFactory(AutoFactory autonFactory) {
        m_autonFactory = autonFactory;
        m_routine = m_autonFactory.newRoutine("auton");
    }

    class AutonCycle {
        public final ReefLocs reefLoc;
        public final EleHeight height;
        public final HPStation hpStation;
        public final ReefHPPair reefHPPair;
        public final HPReefPair hpReefPair;
        Elastic.Notification reefToHPError = 
                    new Elastic.Notification(
                        Elastic.Notification.NotificationLevel.ERROR, 
                        "Invalid reef to HP AutonCycle", 
                        "INVALID REEF-TO-HP: if this auton was a cat, you just killed it."
                    );
        Elastic.Notification hpToReefError = 
                        new Elastic.Notification(
                            Elastic.Notification.NotificationLevel.ERROR, 
                            "Invalid HP to reef AutonCycle", 
                            "INVALID HP-TO-REEF: if this auton was a cat, you just killed it.");
    
        public AutonCycle(ReefLocs _reef, EleHeight _height, HPStation _hp) {
            reefLoc = _reef;
            height = _height;
            hpStation = _hp;
            reefHPPair = new ReefHPPair(reefLoc, hpStation);
            hpReefPair = new HPReefPair(hpStation, reefLoc);
        }

        public boolean isLegit() {
            if(!Trajectories.ReefToHPTrajs.containsKey(reefHPPair)) {
                // banks said that i actually shouldnt write code that crashes the entire program, which im lowk sad abt. was looking forward to it.
                Elastic.sendNotification(reefToHPError);
                return false;
            }
    
            if(!Trajectories.HPToReefTrajs.containsKey(hpReefPair)) {
                // rip DEATH AND DESTRUCTION you were such a good concept in my head. fly high.
                Elastic.sendNotification(hpToReefError);
                return false;
            }
            return true;
        }
    }

    public Command print(String str) {
        return Commands.runOnce(() -> System.out.println(str));
    }

    public Command generateAuton(
        Swerve drivetrain, 
        Superstructure superstructure, 
        StartingLocs startLoc,
        ReefLocs firstReefLoc,
        EleHeight firstHeight, 
        HPStation firstHPStation,
        ArrayList<AutonCycle> cycles
    ) {
        // iterate AutonCycles and validate all HP->Score and Score->HP pairs
        for (AutonCycle autonCycle : cycles) {
            if(!autonCycle.isLegit()) {
                return Commands.print("One of these cycles are WRONG - one of the paths prolly dont exist");
            }
        }

        ArrayList<Command> trajCommands = new ArrayList<Command>();

        // add preload cycle to routine
        AutoTrajectory firstScoreTraj = m_routine.trajectory(Trajectories.StartToReefTrajs.get(new Pair<StartingLocs, ReefLocs>(startLoc, firstReefLoc)));
        AutoTrajectory firstLoadTraj = m_routine.trajectory(Trajectories.ReefToHPTrajs.get(new Pair<ReefLocs, HPStation>(firstReefLoc, firstHPStation)));

        // define events
        firstScoreTraj.done().onTrue(
            Commands.sequence(
                print("first starting to score"),
                superstructure.autonEleToScoringPosReq(firstHeight),
                print("first height achieved"),
                Commands.race(
                    drivetrain.applyRequest(() -> m_brake),
                    print("applying brake"),
                    superstructure.autonScoreReq(),
                    print("done with autonScoreReq")
                ),
                // SOMETHING NEEDS TO REMOVE THE BRAKE
                print("removed brake"),
                superstructure.autonEleToHPReq(),
                print("ele down to HP"),

                print("scoring done")
            )
        );

        // add traj cmds
        trajCommands.add(firstScoreTraj.cmd());
        trajCommands.add(firstLoadTraj.cmd());

        // add other cycles
        for (AutonCycle cycle : cycles) {
            AutoTrajectory hpToReefTraj = m_routine.trajectory(Trajectories.HPToReefTrajs.get(cycle.hpReefPair));
            AutoTrajectory reefToHpTraj = m_routine.trajectory(Trajectories.ReefToHPTrajs.get(cycle.reefHPPair));

            // define events
            hpToReefTraj.done().onTrue(
                Commands.sequence(
                    print("cycles starting to score"),
                    superstructure.autonEleToScoringPosReq(cycle.height),
                    print("cycles height achieved"),
                    Commands.race(
                        drivetrain.applyRequest(() -> m_brake),
                        print("applying brake"),
                        superstructure.autonScoreReq(),
                        print("done with autonScoreReq")
                    ),
                    // SOMETHING NEEDS TO REMOVE THE BRAKE
                    print("removed brake"),
                    superstructure.autonEleToHPReq(),
                    print("cycles ele down to HP"),

                    print("cycles scoring done")
                )
            );

            // add traj cmds
            trajCommands.add(hpToReefTraj.cmd());
            trajCommands.add(reefToHpTraj.cmd());
        }

        // convert arraylist to array
        Command[] cmds = new Command[trajCommands.size() + 1];
        cmds[0] = firstScoreTraj.resetOdometry();
        for (int i = 0; i < trajCommands.size(); i++) {
            cmds[i + 1] = trajCommands.get(i);
        }

        // run commands in order
        m_routine.active().onTrue(
            Commands.sequence(
                cmds
            )
        );

        return m_routine.cmd();
    }

}