package frc.robot.subsystems;

import static frc.robot.Constants.kRumbleIntensity;
import static frc.robot.Constants.kRumbleTimeoutSecs;
import static frc.robot.Constants.RobotK.*;

import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import edu.wpi.first.util.datalog.BooleanArrayLogEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.subsystems.Elevator.EleHeight.*;

import frc.robot.subsystems.Elevator.EleHeight;
import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;
import frc.util.WaltLogger.IntLogger;
import frc.util.WaltLogger.StringLogger;

public class Superstructure {
    private final Coral m_coral;
    private final Elevator m_ele;

    public final EventLoop stateEventLoop = new EventLoop();
    private State m_state = State.IDLE;

    /* requests */
    /* reqs: auton */
    /* reqs: state trans */
    private boolean m_eleToHPStateTransReq = false;
    private boolean m_eleToScoreTransReq = false;
    private boolean m_scoreReq = false;

    /* state transitions */
    /* autoTrgs */
    /* teleopTrgs */
    private final Trigger trg_teleopEleToHPReq;
    private final Trigger trg_teleopL4Req; //TODO: add all 4
    private final Trigger trg_teleopScoreReq;
    /* teleopTrgs: overrides */
    private final Trigger transTrg_toIdleOverrideReq;
    /* Frsies Transition Trigs */
    private final Trigger transTrg_eleToHP = new Trigger(() -> m_eleToHPStateTransReq);
    private final Trigger transTrg_eleNearSetpt; // used for any ele mvmt state
    private final Trigger transTrg_topSensor;
    private final Trigger transTrg_botSensor;
    private final Trigger transTrg_eleToScore = new Trigger(() -> m_eleToScoreTransReq);
    private final Trigger transTrg_scoring = new Trigger(() -> m_scoreReq);

    /* states */
    public final Trigger stateTrg_idle = new Trigger(stateEventLoop, () -> m_state == State.IDLE);
    public final Trigger stateTrg_eleToHP = new Trigger(stateEventLoop, () -> m_state == State.ELE_TO_HP);
    public final Trigger stateTrg_intaking = new Trigger(stateEventLoop, () -> m_state == State.INTAKING);
    public final Trigger stateTrg_intook = new Trigger(stateEventLoop, () -> m_state == State.INTOOK);
    public final Trigger stateTrg_eleToScore = new Trigger(stateEventLoop, () -> m_state == State.ELE_TO_SCORE);
    public final Trigger stateTrg_scoreReady = new Trigger(stateEventLoop, () -> m_state == State.SCORE_READY);
    public final Trigger stateTrg_scoring = new Trigger(stateEventLoop, () -> m_state == State.SCORING);
    public final Trigger stateTrg_scored = new Trigger(stateEventLoop, () -> m_state == State.SCORED);

    public final Trigger stateTrg_eleToClimb = new Trigger(stateEventLoop, () -> m_state == State.ELE_TO_CLIMB);
    public final Trigger stateTrg_climbReady = new Trigger(stateEventLoop, () -> m_state == State.CLIMB_READY);
    public final Trigger stateTrg_climbing = new Trigger(stateEventLoop, () -> m_state == State.CLIMBING);
    public final Trigger stateTrg_climbed = new Trigger(stateEventLoop, () -> m_state == State.CLIMBED);

    /* sm odds & ends */
    private final DoubleConsumer m_driverRumbler;
    private EleHeight m_curHeightReq = HOME;
    private Supplier<EleHeight> m_curHeightReqSupplier = () -> m_curHeightReq;

    /* loggin' */
    private IntLogger log_stateIdx = WaltLogger.logInt(kLogTab, "state idx");
    private StringLogger log_stateName = WaltLogger.logString(kLogTab, "state name");

    private DoubleLogger log_eleReqHeight = WaltLogger.logDouble(kLogTab, "requested height rotations");
    /* logs: state trans */
    private BooleanLogger log_teleopToHPReq = WaltLogger.logBoolean(kLogTab, "TELEOP to HP req");
    private BooleanLogger log_teleopScoreReq = WaltLogger.logBoolean(kLogTab, "TELEOP score req");
    private BooleanLogger log_toIdleOverride = WaltLogger.logBoolean(kLogTab, "to idle override");

    private BooleanLogger log_eleToHPReq = WaltLogger.logBoolean(kLogTab, "ele to HP req");
    private BooleanLogger log_eleAtSetpt = WaltLogger.logBoolean(kLogTab, "ele at setpoint");
    private BooleanLogger log_topSensor = WaltLogger.logBoolean(kLogTab, "top beam break");
    private BooleanLogger log_botSensor = WaltLogger.logBoolean(kLogTab, "bot beam break");
    private BooleanLogger log_eleToScoreReq = WaltLogger.logBoolean(kLogTab, "ele to score lvl req");
    private BooleanLogger log_scoringReq = WaltLogger.logBoolean(kLogTab, "score req");

    public Superstructure(
        Coral coral,
        Elevator ele,
        Trigger eleToHPReq,
        Trigger L4Req,
        Trigger scoreReq,
        Trigger toIdleOverride,
        DoubleConsumer driverRumbler
    ) {
        m_coral = coral;
        m_ele = ele;

        /* teleop trigs */
        trg_teleopEleToHPReq = eleToHPReq;
        trg_teleopL4Req = L4Req;
        trg_teleopScoreReq = scoreReq;
        /* overrides */
        transTrg_toIdleOverrideReq = toIdleOverride;

        /* state change trigs */
        transTrg_eleNearSetpt = new Trigger(() -> m_ele.nearSetpoint());
        transTrg_topSensor = new Trigger(m_coral.trg_topBeamBreak);
        transTrg_botSensor = new Trigger(m_coral.trg_botBeamBreak);

        /* binded things */
        m_driverRumbler = driverRumbler;

        configureRequests();
        configureStateTransitions();
        configureStateActions();
    }

    /* configs */
    private void configureRequests() {
        (trg_teleopEleToHPReq.and(RobotModeTriggers.teleop()))
            .onTrue(Commands.runOnce(() -> m_eleToHPStateTransReq = true));
        (trg_teleopL4Req.and(RobotModeTriggers.teleop()))
            .onTrue(Commands.runOnce(() -> m_eleToScoreTransReq = true)
                .alongWith(Commands.print("TESTING 1")));
        (trg_teleopScoreReq.and(RobotModeTriggers.teleop()))
            .onTrue(Commands.runOnce(() -> m_scoreReq = true));
    }
    
    private void configureStateTransitions() {
        (stateTrg_idle.and(transTrg_eleToHP))
            .onTrue(Commands.runOnce(() -> m_state = State.ELE_TO_HP));
        (stateTrg_eleToHP.debounce(0.02).and(transTrg_eleNearSetpt))
            .onTrue(Commands.runOnce(() -> m_state = State.INTAKING));
        (stateTrg_intaking.and(transTrg_topSensor))
            .onTrue(Commands.runOnce(() -> m_state = State.INTOOK));
        (stateTrg_intook.and(transTrg_eleToScore))
            .onTrue(Commands.runOnce(() -> m_state = State.ELE_TO_SCORE)
            .alongWith(Commands.print("TESTING 2")));
        (stateTrg_eleToScore.debounce(1).and(transTrg_eleNearSetpt))
            .onTrue(Commands.runOnce(() -> m_state = State.SCORE_READY)); 
        (stateTrg_scoreReady.and(transTrg_scoring)) 
            .onTrue(Commands.runOnce(() -> m_state = State.SCORING)
            .alongWith(Commands.print("BAD NO GO")));
        (stateTrg_scoring.and(transTrg_botSensor.negate())) 
            .onTrue(Commands.runOnce(() -> m_state = State.SCORED));
        (stateTrg_scored.debounce(0.02))
            .onTrue(Commands.runOnce(() -> m_state = State.ELE_TO_HP));


        (transTrg_toIdleOverrideReq)
            .onTrue(Commands.runOnce(() -> m_state = State.IDLE));
    }
    }

    private void configureStateActions() {
        stateTrg_idle
            .onTrue(resetEverything());

        stateTrg_eleToHP
            .onTrue(
                Commands.parallel(
                    m_ele.toHeight(() -> HP),
                    Commands.runOnce(() -> m_eleToHPStateTransReq = false)
                )

            );

        stateTrg_intaking
            .onTrue(
                Commands.sequence(
                    m_coral.fastIntake(),
                    Commands.waitUntil(m_coral.trg_topBeamBreak),
                    driverRumble(kRumbleIntensity, kRumbleTimeoutSecs)
                )
            );

        stateTrg_intook
            .onTrue(
                Commands.sequence(
                    m_coral.slowIntake(),
                    Commands.waitUntil(m_coral.trg_botBeamBreak),
                    m_coral.stopCoralMotorCmd()
                )
            );

        stateTrg_eleToScore
            .onTrue(
                Commands.sequence(
                    Commands.print("eleToScore + " + m_curHeightReqSupplier.get()),
                Commands.parallel(
                        eleToScoreCmd(),
                    Commands.runOnce(() -> m_eleToScoreTransReq = false)
                    )
                )
            );

        stateTrg_scoreReady
            .onTrue(
                driverRumble(kRumbleIntensity, kRumbleTimeoutSecs)
                .alongWith(Commands.runOnce(() -> System.out.println("scoreReady"))));

        stateTrg_scoring
            .onTrue(
                Commands.parallel(
                    Commands.sequence(
                        m_coral.score(),
                        Commands.waitUntil(m_coral.trg_botBeamBreak.negate()),
                        m_coral.stopCoralMotorCmd()
                    ),
                    Commands.runOnce(() -> m_scoreReq = false).alongWith(Commands.print("SCORIN"))
                )
            );

        stateTrg_scored
            .onTrue(
                driverRumble(kRumbleIntensity, kRumbleTimeoutSecs)
            );

        // stateTrg_eleToClimb
        //     .onTrue();

        // stateTrg_climbReady
        //     .onTrue(null);

        // stateTrg_climbing
        //     .onTrue(null);

        // stateTrg_climbed
        //     .onTrue(null);
    }

    /* methods that Actually Do Things */
    public Command resetEverything() {
        return Commands.sequence(
            m_coral.stopCoralMotorCmd(),
            m_ele.toHeight(() -> HOME),
            driverRumble(0, kRumbleTimeoutSecs)
        );
    }

    /* elevator things */
    // use in robot.java
    public EleHeight requestEleHeight(Supplier<EleHeight> height) {
        m_curHeightReq = height.get();
        System.out.println("HeightReq: " + m_curHeightReq);
        return m_curHeightReq;
    }

    public Command eleToScoreCmd() {
        if(m_curHeightReqSupplier.get() == L1) {
            return m_ele.toHeight(() -> L1)
            .alongWith(Commands.print("INSIDE eleToScoreCmd L1"));
        } else if(m_curHeightReqSupplier.get() == L2) {
            return m_ele.toHeight(() -> L2)
            .alongWith(Commands.print("INSIDE eleToScoreCmd L2"));
        } else if(m_curHeightReqSupplier.get() == L3) {
            return m_ele.toHeight(() -> L3)
            .alongWith(Commands.print("INSIDE eleToScoreCmd L3"));
        } else if(m_curHeightReqSupplier.get() == L4) {
            return m_ele.toHeight(() -> L4)
                .alongWith(Commands.print("INSIDE eleToScoreCmd L4"));
        } else {
            return Commands.print("invalid height :( wanted " + m_curHeightReqSupplier.get());
        }
    }

    /* to be used in auton */

    /* rumblin' */
    private Command driverRumble(double intensity, double secs) {
        return Commands.startEnd(
           () -> m_driverRumbler.accept(intensity),
           () -> m_driverRumbler.accept(0)
        ).withTimeout(secs);
    }

    /* loggin' */
   public void logState() {
        log_stateIdx.accept(m_state.idx);
        log_stateName.accept(m_state.name);
    }

    public void logRequests() {
        log_teleopToHPReq.accept(trg_teleopEleToHPReq);
        log_teleopScoreReq.accept(trg_teleopScoreReq);
    }

    public void logStateChangeReqs() {
        log_eleToHPReq.accept(transTrg_eleToHP);
        log_eleAtSetpt.accept(transTrg_eleNearSetpt);
        log_topSensor.accept(transTrg_topSensor);
        log_botSensor.accept(transTrg_botSensor);
        log_eleToScoreReq.accept(transTrg_eleToScore);
        log_scoringReq.accept(transTrg_scoring);

        log_toIdleOverride.accept(transTrg_toIdleOverrideReq);
    }

    public void periodic() {
        stateEventLoop.poll();
        logRequests();
        logStateChangeReqs();
        logState();

        log_eleReqHeight.accept(m_curHeightReqSupplier.get().rotations);
    }

    public enum State {
        IDLE(0, "idle"),
        ELE_TO_HP(1, "ele to intake"),
        INTAKING(2, "intaking"),
        INTOOK(3, "intook"),
        ELE_TO_SCORE(4, "ele to score"),
        SCORE_READY(5, "score ready"),
        SCORING(6, "scoring"),
        SCORED(7, "scored"),

        ELE_TO_CLIMB(8, "ele to climb"),
        CLIMB_READY(9, "climb ready"),
        CLIMBING(10, "climbing"),
        CLIMBED(11, "climbed");

        public final int idx;
        public final String name;
  
        private State(int index, String _name) {
            idx = index;
            name = _name;
        }
    }
}
