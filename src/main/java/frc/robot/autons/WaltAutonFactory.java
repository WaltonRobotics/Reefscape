package frc.robot.autons;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.autons.TrajsAndLocs.ReefLocs.REEF_G;
import static frc.robot.autons.TrajsAndLocs.Trajectories.*;

import java.util.ArrayList;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

import frc.robot.Constants.LegacyAutoAlignK;
import frc.robot.Constants.RobotK;
import frc.robot.Constants.SharedAutoAlignK;
import frc.robot.autoalign.LegacyAutoAlign;
import frc.robot.FieldConstants;
import frc.robot.autons.TrajsAndLocs.HPStation;
import frc.robot.autons.TrajsAndLocs.ReefLocs;
import frc.robot.autons.TrajsAndLocs.StartingLocs;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Funnel;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Elevator.EleHeight;
import frc.util.AllianceFlipUtil;
import frc.util.Elastic;
import frc.util.WaltLogger;
import frc.util.WaltLogger.DoubleLogger;
import frc.util.WaltLogger.StringLogger;

public class WaltAutonFactory {
    private final AutoFactory m_autoFactory;
    private AutoRoutine m_routine;
    private final Superstructure m_superstructure;
    private final Elevator m_ele;
    private final Swerve m_drivetrain;
    private final Funnel m_funnel;

    private StartingLocs m_startLoc;
    // all need to have at least 1 thing in them
    private ArrayList<ReefLocs> m_scoreLocs;
    private ArrayList<EleHeight> m_heights; // needs to hv same size as m_scoreLocs
    private ArrayList<HPStation> m_hpStations; // needs to either have same size or one les than m_scoreLocs

    public Timer autonTimer = new Timer();
    private DoubleLogger log_autonTimer = WaltLogger.logDouble(RobotK.kLogTab, "timer", PubSubOption.sendAll(true));
    private StringLogger log_currentPath = WaltLogger.logString(RobotK.kLogTab, "curPath", PubSubOption.sendAll(true));

    private static Command printLater(Supplier<String> stringSup) {
		return Commands.defer(() -> {
			return Commands.print(stringSup.get());
		}, Set.of());
	}

    private Command logTimer(String epochName, Supplier<Timer> timerSup) {
        return printLater(() -> {
            var timer = timerSup.get();
            log_autonTimer.accept(autonTimer.get());
            return epochName + " at " + timer.get() + " s";
        });
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
        Swerve drivetrain,
        Funnel funnel,
        StartingLocs startLoc,
        ArrayList<ReefLocs> scoreLocs,
        ArrayList<EleHeight> heights,
        ArrayList<HPStation> hpStations
    ) {
        m_autoFactory = autoFactory;
        m_routine = m_autoFactory.newRoutine("auton");
        m_superstructure = superstructure;
        m_ele = ele;
        m_drivetrain = drivetrain;
        m_funnel = funnel;

        m_startLoc = startLoc;
        m_scoreLocs = scoreLocs;
        m_heights = heights;
        m_hpStations = hpStations;
    }

    private String getCycleString(ReefLocs reefLoc, EleHeight height, HPStation hp) {
        return reefLoc.str + " -> " + height.toString() + " -> " + hp.str;
    }

    @Override
    public String toString() {
        String name = m_startLoc.name;
        int numScoreLocs = m_scoreLocs.size();
        int numHPStations = m_hpStations.size();

        name += " " + numScoreLocs + " Piece: ";
        if (numScoreLocs == numHPStations) {
          // if the auton ends at an HP Station
          for (int i = 0; i < numScoreLocs; i++) {
              name += getCycleString(m_scoreLocs.get(i), m_heights.get(i), m_hpStations.get(i));
              if (i != numScoreLocs - 1) {
                name += ", ";
              }
          }
        } else {
          // assumes there is one less HP station than scoring loc
          for (int i = 0; i < numHPStations; i++) {
            name += getCycleString(m_scoreLocs.get(i), m_heights.get(i), m_hpStations.get(i));
              if (i != numScoreLocs) {
                name += ", ";
              }
            }
        }

        return name;
    }

    public void startAutonTimer() {
       autonTimer.start();
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

    private ArrayList<Pair<AutoTrajectory, Optional<ReefLocs>>> trajMaker() {
        ArrayList<Pair<AutoTrajectory, Optional<ReefLocs>>> trajsList = new ArrayList<>();
        try {
            for (int i = 0; i < m_scoreLocs.size(); i++) {
                String rToH = ReefToHPTrajs.get(new Pair<ReefLocs, HPStation>(m_scoreLocs.get(i), m_hpStations.get(i)));
                trajsList.add(
                    new Pair<AutoTrajectory, Optional<ReefLocs>>(m_routine.trajectory(rToH), Optional.empty()));
                System.out.println("Adding RtoH Path: " + rToH);
                if (i < m_scoreLocs.size() - 1) {
                    var reefLoc = m_scoreLocs.get(i + 1);
                    String hToR = HPToReefShortTrajs.get(new Pair<HPStation, ReefLocs>(m_hpStations.get(i), reefLoc));
                    trajsList.add(
                        new Pair<AutoTrajectory, Optional<ReefLocs>>(m_routine.trajectory(hToR), Optional.of(reefLoc)));
                        System.out.println("Adding HtoR Path: " + hToR + " with score: " + reefLoc);
                }
            }

            // for .5 autos.
            if(m_hpStations.size() > m_scoreLocs.size()) {
                trajsList.add(
                    new Pair<AutoTrajectory, Optional<ReefLocs>>(m_routine.trajectory(ReefToHPTrajs.get(
                        new Pair<ReefLocs, HPStation>(
                            m_scoreLocs.get(m_scoreLocs.size() - 1),
                            m_hpStations.get(m_hpStations.size() - 1)
                        ))
                ), Optional.empty()));
            }

            return trajsList;
        } catch (Exception e) {
            return trajsList;
        }

    }

    private Command scoreCmd(EleHeight eleHeight) {
        return Commands.sequence(
            m_superstructure.autonScoreReq(),
            logTimer("CoralScored", () -> autonTimer),
            Commands.waitUntil(m_superstructure.getBottomBeamBreak().negate())
            //Commands.waitUntil(m_superstructure.getSimScored())
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

    private Pose2d getReefAutoAlignPose(ReefLocs reefLoc) {
        Pose2d reefPose = FieldConstants.kReefRobotLocationPoseMap.getOrDefault(reefLoc, new Pose2d());
        return AllianceFlipUtil.apply(reefPose);
    }

    private Command autoAlignCommand(Supplier<ReefLocs> reefLocSup) {
        return LegacyAutoAlign.moveToPoseUntilInTimeScaledTolerance(m_drivetrain,
            () -> getReefAutoAlignPose(reefLocSup.get()),
            () -> 1,
            () -> 10 * SharedAutoAlignK.kFieldTranslationTolerance.in(Meters),
            () -> 10 * SharedAutoAlignK.kFieldRotationTolerance.in(Radians));
    }

    public AutoRoutine midAuton() {
        var theTraj = StartToReefShortTrajs.get(new Pair<StartingLocs , ReefLocs>(StartingLocs.MID_G, ReefLocs.REEF_G));
        AutoTrajectory firstScoreTraj = m_routine.trajectory(theTraj);
        System.out.println("Running Path: " + theTraj);

        Command firstCmd = firstScoreTraj.cmd();
        // if (RobotBase.isSimulation()) {
            // firstCmd = firstScoreTraj.resetOdometry().andThen(firstScoreTraj.cmd());
        // }

        m_routine.active().onTrue(
            firstCmd
        );

        firstScoreTraj.done()   
            .onTrue(
                Commands.sequence(
                    Commands.parallel(
                        autoAlignCommand(() -> REEF_G),
                        m_superstructure.autonEleToScoringPosReq(EleHeight.L4)
                    ),
                    scoreCmd(EleHeight.L4)
                )
            );

        return m_routine;
    }

    public AutoRoutine generateAuton() {
        int heightCounter = 0;
        if (doNothing()) {
            return m_routine;
        }

        if (areWeLeaving()) {
            Elastic.sendNotification(leaveStartZoneOnlySadness);
            return leaveOnly();
        }

        if(!notOtherwiseBrokeyChecker()) {
            System.out.println("!!!!!!! BROKE !!!!!!!");
        }

        var theTraj = StartToReefShortTrajs.get(new Pair<StartingLocs , ReefLocs>(m_startLoc, m_scoreLocs.get(0)));
        AutoTrajectory firstScoreTraj = m_routine.trajectory(theTraj);
        System.out.println("Running Initial Path: " + theTraj);

        Command firstCmd = firstScoreTraj.cmd().alongWith(Commands.print("FirstScoreTraj->Running"));
        // if (RobotBase.isSimulation()) {
            // firstCmd = firstScoreTraj.resetOdometry().andThen(firstScoreTraj.cmd());
        // }

        m_routine.active().onTrue(
            firstCmd
        );

        m_routine.active().debounce(0.25).onTrue(
            m_funnel.ejectFlap().asProxy().withTimeout(0.25)
        );

        // normal cycle logic down here
        ArrayList<Pair<AutoTrajectory, Optional<ReefLocs>>> allTheTrajs = trajMaker();

        firstScoreTraj.done()
            .onTrue(
                Commands.sequence(
                    Commands.parallel(
                        autoAlignCommand(() -> m_scoreLocs.get(0)),
                        m_superstructure.autonEleToScoringPosReq(m_heights.get(heightCounter))
                    ),
                    scoreCmd(m_heights.get(heightCounter++)),
                    allTheTrajs.get(0).getFirst().cmd()
                )
            );

        int allTrajIdx = 0;
        while (allTrajIdx < allTheTrajs.size()) {

            Command trajCmd = Commands.none();
            if ((allTrajIdx + 1) < allTheTrajs.size()) {
                trajCmd = allTheTrajs.get(allTrajIdx + 1).getFirst().cmd();
            }

            if (RobotBase.isSimulation()) {
                allTheTrajs.get(allTrajIdx).getFirst().done()
                    .onTrue(Commands.sequence(
                        Commands.waitUntil(() -> m_superstructure.m_state == Superstructure.State.ELE_TO_HP),
                        trajCmd,
                        m_drivetrain.stopCmd(),
                        Commands.print("Running Path: " + trajCmd)
                ));
            } else {
            allTheTrajs.get(allTrajIdx).getFirst().done()
                .onTrue(Commands.sequence(
                    // Commands.waitUntil(m_superstructure.getTopBeamBreak().debounce(0.08)),
                    Commands.waitUntil(m_funnel.trg_atCurrLim.or(m_superstructure.getTopBeamBreak()))
                        .alongWith(Commands.print("funnel detected coral")),
                    trajCmd,
                    m_drivetrain.stopCmd(),
                    Commands.print("Running Path: " + trajCmd)
            ));
            }

            allTrajIdx++;

            if (allTrajIdx > allTheTrajs.size() - 1) {
                break;
            }

            Command nextTrajCmd = Commands.none();
            if (allTrajIdx + 1 < allTheTrajs.size()) {
                nextTrajCmd = allTheTrajs.get(allTrajIdx + 1).getFirst().cmd();
            }

            Command autoAlign = Commands.none();
            AutoTrajectory runningTraj = allTheTrajs.get(allTrajIdx).getFirst();
            Trigger afterPathTrg = runningTraj.done();
            var reefLocOpt = allTheTrajs.get(allTrajIdx).getSecond();
            if (reefLocOpt.isPresent()) {
                autoAlign = autoAlignCommand(() -> reefLocOpt.get());
                // double trajTime = runningTraj.getRawTrajectory().getTotalTime();
                // afterPathTrg = runningTraj.atTimeBeforeEnd(trajTime * 0.3);
            }

            if (RobotBase.isSimulation()) {
                var pathDoneCmd = Commands.sequence(
                    Commands.parallel(
                        autoAlign,
                        m_superstructure.autonEleToScoringPosReq(m_heights.get(heightCounter))
                    ),
                    Commands.waitUntil(() -> m_superstructure.m_state == Superstructure.State.SCORED),
                    scoreCmd(m_heights.get(heightCounter++)),
                    nextTrajCmd,
                    m_drivetrain.stopCmd()
                );

                afterPathTrg.onTrue(
                    pathDoneCmd
                );
            } else {
                var pathDoneCmd = Commands.sequence(
                    Commands.parallel(
                        autoAlign,
                        m_superstructure.autonEleToScoringPosReq(m_heights.get(heightCounter))
                    ),
                    Commands.waitUntil(m_superstructure.getBottomBeamBreak()),
                    scoreCmd(m_heights.get(heightCounter++)),
                    nextTrajCmd,
                    m_drivetrain.stopCmd()
                );

                afterPathTrg.onTrue(
                    pathDoneCmd
                );
            }

            allTrajIdx++;
        }

        return m_routine;
    }
}