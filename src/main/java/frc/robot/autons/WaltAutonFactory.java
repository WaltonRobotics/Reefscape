package frc.robot.autons;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.autons.TrajsAndLocs.Trajectories.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

import frc.robot.Constants.RobotK;
import frc.robot.autons.TrajsAndLocs.HPStation;
import frc.robot.autons.TrajsAndLocs.ReefLocs;
import frc.robot.autons.TrajsAndLocs.StartingLocs;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Elevator.EleHeight;
import frc.util.Elastic;
import frc.util.WaltLogger;
import frc.util.WaltLogger.DoubleLogger;

public class WaltAutonFactory {
    private final AutoFactory m_autoFactory;
    private AutoRoutine m_routine;
    private final Superstructure m_superstructure;
    private final Elevator m_ele;
    private final Swerve m_drivetrain;

    private final StartingLocs m_startLoc;
    // all need to have at least 1 thing in them
    private final ArrayList<ReefLocs> m_scoreLocs;
    private final ArrayList<EleHeight> m_heights; // needs to hv same size as m_scoreLocs
    private final ArrayList<HPStation> m_hpStations; // needs to either have same size or one les than m_scoreLocs

    int heightCounter = 0;

    public Timer autonTimer = new Timer();
    private DoubleLogger log_autonTimer = WaltLogger.logDouble(RobotK.kLogTab, "timer");

    private static Command printLater(Supplier<String> stringSup) {
		return Commands.defer(() -> {
			return Commands.print(stringSup.get());
		}, Set.of());
	}

    private Command logTimer(String epochName, Supplier<Timer> timerSup) {
		return Commands.defer(() -> {
			var timer = timerSup.get();
            log_autonTimer.accept(autonTimer.get());
			return printLater(() -> epochName + " at " + timer.get() + " s");
		}, Set.of());
	}
    
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
        Elevator ele,
        AutoFactory autoFactory,
        Superstructure superstructure,
        Swerve drivetrain
    ) {
        m_autoFactory = autoFactory;
        m_drivetrain = drivetrain;
        m_ele = ele;
        m_superstructure = superstructure;

        m_startLoc = StartingLocs.LEFT;
        m_scoreLocs = new ArrayList<>();
        m_heights = new ArrayList<>();
        m_hpStations = new ArrayList<>();

    }

    public WaltAutonFactory(
        Elevator ele,
        AutoFactory autoFactory, 
        Superstructure superstructure,
        Swerve drivetrain,
        StartingLocs startLoc,
        ArrayList<ReefLocs> scoreLocs,
        ArrayList<EleHeight> heights,
        ArrayList<HPStation> hpStations,
        boolean pushTime
    ) {
        m_autoFactory = autoFactory;
        m_routine = m_autoFactory.newRoutine("auton"); 
        m_superstructure = superstructure;
        m_ele = ele;
        m_drivetrain = drivetrain;

        m_startLoc = startLoc;
        m_scoreLocs = scoreLocs;
        m_heights = heights;
        m_hpStations = hpStations;
    }

    private boolean doNothing() {
        if (m_scoreLocs.size() == 0 && m_heights.size() == 0 && m_hpStations.size() == 0) {
            return true;
        } else {
            return false;
        }
    }

    private boolean areWeLeaving() {
        if (m_scoreLocs.size() == 0 && m_heights.size() == 0 && m_hpStations.size() == 1) { // have a hp station to act as a flag for leaving or staying still
            return true;
        } else {
            return false;
        }
    }

    private boolean onlyPreload() {
        if (m_scoreLocs.size() == 1 && m_heights.size() == 1 && m_hpStations.size() == 0) {
            return true;
        } else {
            return false;
        }
    }

    private boolean notOtherwiseBrokeyChecker() {
        if(m_scoreLocs.size() != m_heights.size()) {
            return false;
        } else if (m_hpStations.size() != m_scoreLocs.size() && m_hpStations.size() != m_scoreLocs.size() - 1) {
            return false;
        } else {
            return true;
        }
    }

    private ArrayList<WaltAutonCycle> cycleMaker(boolean isShort) {
        var reefToHpMap = ReefToHPTrajs;
        var hpToReefMap = isShort ? HPToReefShortTrajs : HPToReefTrajs;

        ArrayList<WaltAutonCycle> cycleList = new ArrayList<>();

        try {
            for (int i = 0; i < m_hpStations.size(); i++) {
                String rToHName = reefToHpMap.get(new Pair<ReefLocs, HPStation>(m_scoreLocs.get(i), m_hpStations.get(i)));
                var rToHTraj = m_routine.trajectory(rToHName);

                // either do or don't add a ReefCycle
                Optional<ReefCycle> reefCycleOpt = Optional.empty();
                if (i < m_scoreLocs.size() - 1) {
                    String hToRName = hpToReefMap.get(new Pair<HPStation, ReefLocs>(m_hpStations.get(i), m_scoreLocs.get(i + 1)));
                    var hToRTraj = m_routine.trajectory(hToRName);
                    reefCycleOpt = Optional.of(new ReefCycle(hToRTraj, m_scoreLocs.get(i), m_heights.get(i)));
                }

                var fullCycle = new WaltAutonCycle(rToHTraj, reefCycleOpt);
                cycleList.add(fullCycle);
            }
            return cycleList;
        } catch (Exception ex) {
            return cycleList;
        }
    }

    private Command scoreCmd(EleHeight eleHeight) {
        return Commands.sequence(
            m_ele.externalWaitUntilHomed(),
            m_superstructure.autonEleToScoringPosReq(eleHeight),
            m_superstructure.autonScoreReq(),
            m_superstructure.simHasCoralToggle(),
            logTimer("CoralScored", () -> autonTimer),
            Commands.waitUntil(m_superstructure.getBottomBeamBreak().negate())
        );
    }

    

    public AutoRoutine leaveOnly() {
        AutoRoutine leaveAuto = m_autoFactory.newRoutine("leave only");
        //TODO: [before we vision] have sophomores make an unjanky version of this
        AutoTrajectory leave = m_routine.trajectory("One_Meter");
        leaveAuto.active().onTrue(
            Commands.sequence(
                Commands.runOnce(() -> autonTimer.restart()),
                leave.cmd()
            )
        );

        return leaveAuto;
    }

    private Command autoAlignCmd(ReefLocs scoreLoc, boolean useAutoAlign) {
        if (!useAutoAlign) {
            return Commands.none();
        }

        return m_drivetrain.moveToPose(scoreLoc.getIdealScoringPose());
    }

    final record ReefCycle(
        AutoTrajectory traj,
        ReefLocs loc,
        EleHeight height
    ) {}

    final record WaltAutonCycle(
        AutoTrajectory sourceTraj,
        Optional<ReefCycle> reefCycle
    ) {}

    public void executeChainedCyclesLoopStrict(
        Trigger startTrigger,
        ReefCycle firstReefCycle,
        List<WaltAutonCycle> allTheCycles,
        boolean isShort
    ) {
        if (allTheCycles.isEmpty()) return;
    
        // --- Run FIRST ReefCycle on m_routine.active() ---
        WaltAutonCycle firstFullCycle = allTheCycles.get(0);
        AutoTrajectory firstFullCycleSource = firstFullCycle.sourceTraj();
    
        startTrigger.onTrue(
            Commands.sequence(
                Commands.runOnce(() -> autonTimer.restart()),
                firstReefCycle.traj.cmd()
            )
        );

        firstReefCycle.traj.done().onTrue(
            Commands.sequence(
                autoAlignCmd(firstReefCycle.loc, isShort),
                m_drivetrain.stopCmd(),
                Commands.waitUntil(m_superstructure.getBottomBeamBreak()),
                scoreCmd(firstReefCycle.height()),
                firstFullCycleSource.cmd(),
                m_drivetrain.stopCmd()
            )
        );
    
        // --- First sourceTraj.done() → get coral + score setup ---
        firstFullCycleSource.done().onTrue(
            Commands.sequence(
                m_superstructure.simHasCoralToggle(),
                Commands.waitUntil(
                    m_superstructure.getTopBeamBreak().debounce(0.08)
                        .or(m_superstructure.simTrg_hasCoral)
                ),
                firstFullCycle.reefCycle().map(r -> r.traj().cmd()).orElse(Commands.none()),
                m_drivetrain.stopCmd()
            )
        );
    
        // --- Loop through remaining cycles ---
        for (int i = 1; i < allTheCycles.size(); i++) {
            WaltAutonCycle prevCycle = allTheCycles.get(i - 1);
            WaltAutonCycle currentCycle = allTheCycles.get(i);
    
            AutoTrajectory sourceTraj = currentCycle.sourceTraj();
            Optional<ReefCycle> currentReefOpt = currentCycle.reefCycle();
            Optional<ReefCycle> prevReefOpt = prevCycle.reefCycle();
    
            // At reef arrival (prevCycle.done()) → score, move to HP
            prevReefOpt.ifPresent(prevReef -> {
                prevReef.traj.done().onTrue(
                    Commands.sequence(
                        Commands.waitUntil(m_superstructure.getBottomBeamBreak()),
                        scoreCmd(prevReef.height()),
                        sourceTraj.cmd(),
                        m_drivetrain.stopCmd()
                    )
                );
            });
    
            // At HP arrival (sourceTraj.done()) → get coral, move to reef
            sourceTraj.done().onTrue(
                Commands.sequence(
                    m_superstructure.simHasCoralToggle(),
                    Commands.waitUntil(
                        m_superstructure.getTopBeamBreak().debounce(0.08)
                            .or(m_superstructure.simTrg_hasCoral)
                    ),
                    currentReefOpt.map(r -> r.traj().cmd()).orElse(Commands.none()),
                    currentReefOpt.map(r -> autoAlignCmd(r.loc(), isShort)).orElse(Commands.none()),
                    m_drivetrain.stopCmd()
                )
            );
        }
    }
    

    public AutoRoutine cycleRoutineMaker(boolean isShort) {
        var startToReefMap = isShort ? StartToReefShortTrajs : StartToReefTrajs;
        var startScoreLoc = m_scoreLocs.get(0);
        var startTrajName = startToReefMap.get(new Pair<StartingLocs , ReefLocs>(m_startLoc, startScoreLoc));
        AutoTrajectory firstScoreTraj = m_routine.trajectory(startTrajName);

        ReefCycle firstCycle = new ReefCycle(firstScoreTraj, startScoreLoc, m_heights.get(0));
        List<WaltAutonCycle> allTheCycles = cycleMaker(isShort);
        executeChainedCyclesLoopStrict(m_routine.active(), firstCycle, allTheCycles, isShort);

        return m_routine;
    }
}