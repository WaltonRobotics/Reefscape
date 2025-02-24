package frc.robot.subsystems;

import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Constants.*;

import frc.robot.subsystems.Elevator.EleHeight;
import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;
import frc.util.WaltLogger.IntLogger;

public class Superstructure {
    private final static String kLogTab = "Superstructure";

    private final Coral m_coral;
    private final Elevator m_ele;

    private final DoubleConsumer m_driverRumbler, m_manipRumbler;

    private State m_state;

    // reqs
    private boolean autonIntakeReq = false;
    private boolean autonScoreEleReq = false;
    private boolean autonScoreReq = false;

    private boolean preloadOverride = false;
    private boolean autonIntakeOverride = false;
    private boolean autonScoreOverride = false;

    private boolean eleHeightReq = false;

    public final EventLoop stateEventLoop = new EventLoop();

    private final Trigger trg_autonIntakeReq = new Trigger(() -> autonIntakeReq);
    private final Trigger trg_teleopIntakeReq;
    private final Trigger trg_topSensor;
    private final Trigger trg_botSensor;
    private final Trigger trg_eleHeightReq = new Trigger(() -> eleHeightReq); // ready to check currHeight and move
    private final Trigger trg_autonScoreReq = new Trigger(() -> autonScoreReq);
    private final Trigger trg_teleopScoreReq;

    // TODO: clean up logging
    private final BooleanLogger log_autonIntakeReq = new BooleanLogger(kLogTab, "autonIntakeReq");
    private final BooleanLogger log_teleopIntakeReq = new BooleanLogger(kLogTab, "teleopIntakeReq");
    private final BooleanLogger log_topSensor = new BooleanLogger(kLogTab, "topSensor");
    private final BooleanLogger log_botSensor = new BooleanLogger(kLogTab, "botSensor");
    private final BooleanLogger log_autonScoreEleReq = new BooleanLogger(kLogTab, "autonScoreEleReq");
    private final BooleanLogger log_eleHeightReq = new BooleanLogger(kLogTab, "eleHeightReq");
    private final BooleanLogger log_autonScoreReq = new BooleanLogger(kLogTab, "autonScoreReq");
    private final BooleanLogger log_teleopScoreReq = new BooleanLogger(kLogTab, "teleopScoreReq");

    private final Trigger trg_climbUpReq;
    private final Trigger trg_climbDownReq;

    private final BooleanLogger log_climbUpReq = new BooleanLogger(kLogTab, "climbUpReq");
    private final BooleanLogger log_climbDownReq = new BooleanLogger(kLogTab, "climbDownReq");

    private final Trigger trg_eleNearSetpoint;

    private final BooleanLogger log_eleNearSetpoint = new BooleanLogger(kLogTab, "eleNearSetpt");

    private final Trigger trg_teleopIntakeStateOverride;
    private final Trigger trg_teleopScoreStateOverride;
    private final Trigger trg_toIdleStateOverride;

    private final BooleanLogger log_teleopIntakeStateOverride = new BooleanLogger(kLogTab, "teleopIntakeStateOverride");
    private final BooleanLogger log_teleopScoreStateOverride = new BooleanLogger(kLogTab, "teleopScoreStateOverride");
    private final BooleanLogger log_toIdleStateOverride = new BooleanLogger(kLogTab, "toIdleStateOverride");

    private final Trigger trg_preloadOverride = new Trigger(() -> preloadOverride);
    private final Trigger trg_autonIntakeOverride = new Trigger(() -> autonIntakeOverride);
    private final Trigger trg_autonScoreOverride = new Trigger(() -> autonScoreOverride);

    private final BooleanLogger log_preloadOverride = new BooleanLogger(kLogTab, "preloadOverride");
    private final BooleanLogger log_autonIntakeOverride = new BooleanLogger(kLogTab, "autonIntakeOverride");
    private final BooleanLogger log_autonScoreOverride = new BooleanLogger(kLogTab, "autonScoreOverride");

    public final Trigger stateTrg_idle = new Trigger(stateEventLoop, () -> m_state == State.IDLE);
    public final Trigger stateTrg_eleToIntake = new Trigger(stateEventLoop, () -> m_state == State.ELE_TO_INTAKE);
    public final Trigger stateTrg_intaking = new Trigger(stateEventLoop, () -> m_state == State.INTAKING);
    public final Trigger stateTrg_intook = new Trigger(stateEventLoop, () -> m_state == State.INTOOK);
    public final Trigger stateTrg_eleToScore = new Trigger(stateEventLoop, () -> m_state == State.ELE_TO_SCORE);
    public final Trigger stateTrg_scoreReady = new Trigger(stateEventLoop, () -> m_state == State.SCORE_READY);
    public final Trigger stateTrg_score = new Trigger(stateEventLoop, () -> m_state == State.SCORING);
    public final Trigger stateTrg_scored = new Trigger(stateEventLoop, () -> m_state == State.SCORED);

    public final BooleanLogger log_idle = new BooleanLogger(kLogTab, "idle");
    public final BooleanLogger log_eleToIntake = new BooleanLogger(kLogTab, "eleToIntake");
    public final BooleanLogger log_intaking = new BooleanLogger(kLogTab, "intaking");
    public final BooleanLogger log_intook = new BooleanLogger(kLogTab, "intook");
    public final BooleanLogger log_eleToScore = new BooleanLogger(kLogTab, "eleToScore");
    public final BooleanLogger log_scoreReady = new BooleanLogger(kLogTab, "scoreReady");
    public final BooleanLogger log_score = new BooleanLogger(kLogTab, "score");
    public final BooleanLogger log_scored = new BooleanLogger(kLogTab, "scored");

    public final Trigger stateTrg_eleToClimb = new Trigger(stateEventLoop, () -> m_state == State.ELE_TO_CLIMB);
    public final Trigger stateTrg_climbReady = new Trigger(stateEventLoop, () -> m_state == State.CLIMB_READY);
    public final Trigger stateTrg_climbing = new Trigger(stateEventLoop, () -> m_state == State.CLIMBING);
    public final Trigger stateTrg_climbed = new Trigger(stateEventLoop, () -> m_state == State.CLIMBED);

    private final Trigger stateTrgExt_haveCoral = stateTrg_intook.or(stateTrg_eleToScore).or(stateTrg_scoreReady);

    private EleHeight m_curHeightReq = EleHeight.HOME;

    private IntLogger log_state = WaltLogger.logInt(kLogTab, "state");
    private DoubleLogger log_eleReqHeight = WaltLogger.logDouble(kLogTab, "eleReqHeight");

    public Superstructure(
        Coral coral, Elevator ele, 
        Trigger teleopIntakeReq,
        Trigger teleopScoreReq, 
        Trigger teleopIntakeStateOverride,
        Trigger teleopScoreEleOverride,
        Trigger toIdleStateOverride,
        Trigger climbUpReq,
        Trigger climbDownReq,
        DoubleConsumer driverRumble,
        DoubleConsumer manipRumble
    ) {
        m_coral = coral;
        m_ele = ele;

        m_driverRumbler = driverRumble;
        m_manipRumbler = manipRumble;

        trg_teleopIntakeReq = teleopIntakeReq;
        trg_topSensor = m_coral.trg_topBeamBreak;
        trg_botSensor = m_coral.trg_botBeamBreak;
        trg_eleNearSetpoint = new Trigger(() -> m_ele.nearSetpoint());

        trg_teleopIntakeStateOverride = teleopIntakeStateOverride;
        trg_teleopScoreReq = teleopScoreReq;
        trg_teleopScoreStateOverride = teleopScoreEleOverride;
        trg_toIdleStateOverride = toIdleStateOverride;

        trg_climbUpReq = climbUpReq;
        trg_climbDownReq = climbDownReq;
        
        m_state = State.IDLE;

        configureAutonTrgs();
        configureTeleopTrgs();
        configureStateTransitions();
        configureStateActions();
    }

    private void configureAutonTrgs() {
        // these are treated kinda differently in auton than in teleop so i differentiated them again
        (stateTrg_idle.and(trg_autonIntakeReq).and(RobotModeTriggers.autonomous()))
            .onTrue(Commands.runOnce(() -> m_state = State.ELE_TO_INTAKE));
        (stateTrg_scoreReady.and(trg_autonScoreReq).and(RobotModeTriggers.autonomous()))
            .onTrue(Commands.runOnce(() -> m_state = State.SCORING));
        
        // overrides
        (trg_preloadOverride.and(stateTrg_idle).and(RobotModeTriggers.autonomous()))
            .onTrue(Commands.runOnce(() -> m_state = State.INTOOK));
        (trg_autonIntakeOverride.and(RobotModeTriggers.autonomous()))
            .onTrue(Commands.runOnce(() -> m_state = State.INTAKING));
        (trg_autonScoreOverride.and(RobotModeTriggers.autonomous()))
            .onTrue(Commands.runOnce(() -> m_state = State.SCORING));
    }

    private void configureTeleopTrgs() {
        (stateTrg_idle.and(trg_teleopIntakeReq).and(RobotModeTriggers.teleop()))
            .onTrue(Commands.runOnce(() -> m_state = State.ELE_TO_INTAKE));
        (stateTrg_scoreReady.and(trg_teleopScoreReq).and(RobotModeTriggers.teleop()))
            .onTrue(Commands.runOnce(() -> m_state = State.SCORING));
        (stateTrg_idle.and(trg_climbUpReq).and(RobotModeTriggers.teleop()))
            .onTrue(Commands.runOnce(() -> m_state = State.ELE_TO_CLIMB));

        (stateTrg_eleToClimb.and(trg_eleNearSetpoint).and(RobotModeTriggers.teleop()))
            .onTrue(Commands.runOnce(() -> m_state = State.CLIMB_READY));
        (stateTrg_climbReady.and(trg_climbDownReq).and(RobotModeTriggers.teleop()))
            .onTrue(Commands.runOnce(() -> m_state = State.CLIMBING));
        (stateTrg_climbing.and(trg_eleNearSetpoint).and(RobotModeTriggers.teleop()))
            .onTrue(Commands.runOnce(() -> m_state = State.CLIMBED));

        // overrides
        (trg_toIdleStateOverride.and(RobotModeTriggers.teleop()))
            .onTrue(Commands.runOnce(() -> m_state = State.IDLE));
        (trg_teleopIntakeStateOverride.and(RobotModeTriggers.teleop()))
            .onTrue(Commands.runOnce(() -> m_state = State.ELE_TO_INTAKE));
        (trg_teleopScoreStateOverride.and(RobotModeTriggers.teleop()))
            .onTrue(Commands.runOnce(() -> m_state = State.ELE_TO_SCORE));
    }

    private void configureStateTransitions() {
         // reset score req 
         trg_eleHeightReq.debounce(0.02, DebounceType.kRising)
            .onTrue(Commands.runOnce(() -> {eleHeightReq = false;}));

        (stateTrg_eleToIntake.debounce(0.02).and(trg_eleNearSetpoint))
            .onTrue(Commands.runOnce(() -> m_state = State.INTAKING));
        (stateTrg_intaking.and(trg_topSensor))
            .onTrue(Commands.runOnce(() -> m_state = State.INTOOK));
        (stateTrg_intook.and(trg_eleHeightReq))
            .onTrue(Commands.runOnce(() -> m_state = State.ELE_TO_SCORE));
        (stateTrg_eleToScore.and(trg_eleNearSetpoint))
            .onTrue(Commands.runOnce(() -> m_state = State.SCORE_READY));
        (stateTrg_score.and(trg_botSensor.negate()))
            .onTrue(Commands.runOnce(() -> m_state = State.SCORED));
        (stateTrg_scored.and(trg_toIdleStateOverride.negate()))
            .onTrue(Commands.runOnce(() -> m_state = State.ELE_TO_INTAKE));
    }

    private void configureStateActions() {
        (stateTrg_idle)
            .onTrue(
                Commands.sequence(
                    m_coral.stopCoralMotorCmd(),
                    m_ele.toHeight(() -> EleHeight.HOME),
                    driverRumble(0, kRumbleTimeoutSecs),
                    manipRumble(0, kRumbleTimeoutSecs)
                )
            );

        (stateTrg_eleToIntake)
            .onTrue(m_ele.toHeight(() -> EleHeight.HP));
        (stateTrg_intaking)
            .onTrue(
                Commands.sequence(
                    m_coral.fastIntake(),
                    Commands.waitUntil(m_coral.trg_topBeamBreak),
                    driverRumble(kRumbleIntensity, kRumbleTimeoutSecs)
                )
            );
        (stateTrg_intook)
            .onTrue(
                Commands.sequence(
                    m_coral.slowIntake(),
                    Commands.waitUntil(m_coral.trg_botBeamBreak),
                    m_coral.stopCoralMotorCmd(),
                    Commands.print("niah niah")
                )
            );
        (stateTrgExt_haveCoral).and(trg_eleHeightReq)
            .onTrue(
                Commands.sequence(
                    m_ele.toHeight(() -> m_curHeightReq)
                ));
        (stateTrg_scoreReady)
            .onTrue(
                driverRumble(kRumbleIntensity, kRumbleTimeoutSecs)
             );
        (stateTrg_score)
            .onTrue(m_coral.score());
        (stateTrg_scored)
            .onTrue(
                m_coral.setCoralMotorActionCmd(0)
            );
        
        (stateTrg_eleToClimb)
            .onTrue(m_ele.toHeight(() -> EleHeight.CLIMB_UP));
        (stateTrg_climbReady)
            .onTrue(manipRumble(kRumbleIntensity, kRumbleTimeoutSecs));
        (stateTrg_climbing)
            .onTrue(m_ele.toHeight(() -> EleHeight.CLIMB_DOWN));
        (stateTrg_climbed)
            .onTrue(Commands.print("yippeee we done we done"));
    }

    private Command driverRumble(double intensity, double secs) {
        return Commands.startEnd(
           () -> m_driverRumbler.accept(intensity),
           () -> m_driverRumbler.accept(0)
        ).withTimeout(secs);
    }

    private Command manipRumble(double intensity, double secs) {
        return Commands.startEnd(
            () -> m_manipRumbler.accept(intensity),
            () -> m_driverRumbler.accept(0)
        ).withTimeout(secs);
    }

    public void requestIsPreload(boolean preload) {
        preloadOverride = preload;
    }

    public Command autonRequestEleToScore(EleHeight height) {
        return Commands.runOnce(() -> {
            m_curHeightReq = height;
            autonScoreEleReq = true;
        });
    }

    public Command autonScoreReq() {
        return Commands.runOnce(() -> autonScoreReq = true);
    }

    public Command requestEleHeight(Supplier<EleHeight> height, boolean auton) {
        return Commands.runOnce(() -> {
            m_curHeightReq = height.get();
            System.out.println("HeightReq - " + m_curHeightReq + ", auton: " + auton);
            if (auton) { autonIntakeReq = true; }
            else { eleHeightReq = true; }
            log_eleReqHeight.accept(m_curHeightReq.rotations);
        });
    }

    public Command autonRequestToIntake() {
        return requestEleHeight(() -> EleHeight.HP, true);
    }

    public Command teleopIntakeRequest() {
        return requestEleHeight(() -> EleHeight.HP, false);
    }

    public Command requestEleHeight(Supplier<EleHeight> height) {
        return requestEleHeight(height, false);
    }

    public void logState() {
        log_state.accept(m_state.idx);
    }

    public void logTriggers() {
        log_teleopIntakeReq.accept(trg_teleopIntakeReq);
        log_botSensor.accept(trg_botSensor);
    }

    public void periodic() {
        stateEventLoop.poll();
        logTriggers();
        logState();
        log_eleReqHeight.accept(m_curHeightReq.rotations);
        log_eleHeightReq.accept(trg_eleHeightReq.getAsBoolean());
    }

    public enum State {
        IDLE(0),
        ELE_TO_INTAKE(1),
        INTAKING(2),
        INTOOK(3),
        ELE_TO_SCORE(4),
        SCORE_READY(5),
        SCORING(6),
        SCORED(7),

        ELE_TO_CLIMB(8),
        CLIMB_READY(9),
        CLIMBING(10),
        CLIMBED(11);

        public final int idx;
  
        private State(int index) {
            idx = index;
        }
    }
}
