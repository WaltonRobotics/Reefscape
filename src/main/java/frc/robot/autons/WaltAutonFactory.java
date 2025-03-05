package frc.robot.autons;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import static frc.robot.autons.TrajsAndLocs.ReefLocs.REEF_H;
import static frc.robot.autons.TrajsAndLocs.Trajectories.*;

import java.util.ArrayList;
import frc.robot.autons.TrajsAndLocs.HPStation;
import frc.robot.autons.TrajsAndLocs.ReefLocs;
import frc.robot.autons.TrajsAndLocs.StartingLocs;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Elevator.EleHeight;
import frc.util.Elastic;

public class WaltAutonFactory {
    private final AutoFactory m_autoFactory;
    private AutoRoutine m_routine;
    // private final Superstructure m_superstructure;

    private StartingLocs m_startLoc;
    // all need to have at least 1 thing in them
    private ArrayList<ReefLocs> m_scoreLocs;
    private ArrayList<EleHeight> m_heights; // needs to hv same size as m_scoreLocs
    private ArrayList<HPStation> m_hpStations; // needs to either have same size or one les than m_scoreLocs

    int heightCounter = 0;
    
    private Elastic.Notification leaveStartZoneOnlySadness = 
        new Elastic.Notification(
            Elastic.Notification.NotificationLevel.ERROR, 
            "Why We No Score :(", 
            "Just FYI, we're only going to move forward"
        );
    private Elastic.Notification youMessedUpInAutonChooser = 
        new Elastic.Notification(
            Elastic.Notification.NotificationLevel.ERROR, 
            "Can't run full path", 
            "ArrayList sizes didnt match up for reef, height, and HP. Will run as much of the path as possible"
        );

    public WaltAutonFactory(
        AutoFactory autoFactory, 
        // Superstructure superstructure,
        StartingLocs startLoc,
        ArrayList<ReefLocs> scoreLocs,
        ArrayList<EleHeight> heights,
        ArrayList<HPStation> hpStations
    ) {
        m_autoFactory = autoFactory;
        m_routine = m_autoFactory.newRoutine("auton");
        // m_superstructure = superstructure;

        m_startLoc = startLoc;
        m_scoreLocs = scoreLocs;
        m_heights = heights;
        m_hpStations = hpStations;
    }

    private boolean weAtLeastScoreOneChecker() {
        if(m_scoreLocs.size() >= 1 && m_heights.size() >= 1) {
            return true;
        } else {
            return false;
        }
    }

    private boolean notOtherwiseBrokeyChecker() {
        if(m_scoreLocs.size() != m_heights.size()) {
            return false;
        } else if (m_hpStations.size() != m_scoreLocs.size() && m_hpStations.size() != m_scoreLocs.size() + 1) {
            return false;
        } else {
            return true;
        }
    }

    private ArrayList<AutoTrajectory> trajMaker() {
        ArrayList<AutoTrajectory> trajsList = new ArrayList<>();
        for (int i = 0; i < m_scoreLocs.size(); i++) {
            String rToH = ReefToHPTrajs.get(new Pair<ReefLocs, HPStation>(m_scoreLocs.get(i), m_hpStations.get(i)));
            trajsList.add(m_routine.trajectory(rToH));
            if (i < m_scoreLocs.size() - 1) {
                String hToR = HPToReefTrajs.get(new Pair<HPStation, ReefLocs>(m_hpStations.get(i), m_scoreLocs.get(i + 1)));
                trajsList.add(m_routine.trajectory(hToR));
                System.out.println(rToH);
            }
        }

        // for .5 autos.
        if(m_hpStations.size() > m_scoreLocs.size()) {
            trajsList.add(m_routine.trajectory(ReefToHPTrajs.get(
                new Pair<ReefLocs, HPStation>(
                    m_scoreLocs.get(m_scoreLocs.size() - 1), 
                    m_hpStations.get(m_hpStations.size() - 1)
                )
            )));
        }

        return trajsList;
    }

    private Command scoreCmd(EleHeight eleHeight) {
        return Commands.waitSeconds(5);
        // return Commands.sequence(
        //     m_superstructure.autonEleToScoringPosReq(eleHeight),
        //     m_superstructure.autonScoreReq(),
        //     Commands.waitUntil(m_superstructure.getBottomBeamBreak().negate())
        // );
    }

    private AutoRoutine leaveOnly() {
        AutoRoutine leaveAuto = m_autoFactory.newRoutine("leave only");
        //TODO: [before we vision] have sophomores make an unjanky version of this
        AutoTrajectory leave = m_routine.trajectory(StartToReefTrajs.get(new Pair<StartingLocs , ReefLocs>(StartingLocs.MID, REEF_H)));
        leaveAuto.active().onTrue(
            Commands.sequence(
                leave.resetOdometry(),
                leave.cmd()
            )
        );

        return leaveAuto;
    }

    public AutoRoutine generateAuton() {
        if(!weAtLeastScoreOneChecker()) {
            Elastic.sendNotification(leaveStartZoneOnlySadness);   
            return leaveOnly();
        }

        if(!notOtherwiseBrokeyChecker()) {
            System.out.println("!!!!!!! BROKE !!!!!!!");
        }

        var theTraj = StartToReefTrajs.get(new Pair<StartingLocs , ReefLocs>(m_startLoc, m_scoreLocs.get(0)));
        AutoTrajectory firstScoreTraj = m_routine.trajectory(theTraj);
        System.out.println("Running Path: " + theTraj);
        ArrayList<AutoTrajectory> allTheTrajs = trajMaker();

        m_routine.active().onTrue(
            Commands.sequence(
                firstScoreTraj.resetOdometry(),
                firstScoreTraj.cmd()
            )
        );

        firstScoreTraj.done()
            .onTrue(
                Commands.sequence(
                    scoreCmd(m_heights.get(heightCounter)),
                    Commands.parallel(
                        Commands.runOnce(() -> heightCounter++),
                        // benton said to forget the hp station
                        allTheTrajs.get(0).cmd() //takes you to the HP
                    )
                )
            );
        
        int allTrajIdx = 0;
        while (allTrajIdx < allTheTrajs.size()) {
            System.out.println("in the while loop");
            
            Command trajCmd = Commands.none();
            if ((allTrajIdx + 1) < allTheTrajs.size()) {
                System.out.println("to reef cycle called");
                trajCmd = allTheTrajs.get(allTrajIdx + 1).cmd();
            }

            allTheTrajs.get(allTrajIdx).done()
                .onTrue(Commands.sequence(
                    // Commands.print("b4 checking if bottom beam breaks"),
                    // Commands.waitUntil(m_superstructure.getBottomBeamBreak()),
                    // Commands.print("Bottom beam break has broken"),
                    trajCmd,
                    Commands.print("traj command sent")
                ));

            allTrajIdx++;
            
            if (allTrajIdx > allTheTrajs.size() - 1) {
                System.out.println("sad times");
                break;
            }

            Command nextTrajCmd = Commands.none();
            if (allTrajIdx + 1 < allTheTrajs.size()) {
                System.out.println("to hp called");
                nextTrajCmd = allTheTrajs.get(allTrajIdx + 1).cmd();
            }

            allTheTrajs.get(allTrajIdx).done().onTrue(
                Commands.sequence(
                    scoreCmd(m_heights.get(heightCounter)),
                    Commands.parallel(
                        Commands.runOnce(() -> heightCounter++),
                        nextTrajCmd,
                        Commands.print("next traj command sent")
                    )
                )
            );
        }

        return m_routine;
    }
}