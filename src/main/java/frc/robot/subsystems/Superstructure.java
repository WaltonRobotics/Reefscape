package frc.robot.subsystems;

import static frc.robot.Constants.Coralk.kCoralSpeed;

import java.util.function.DoubleConsumer;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Constants.*;
import static frc.robot.Constants.AlgaeK.kLogTab;

import frc.robot.subsystems.Elevator.EleHeight;
import frc.util.WaltLogger;
import frc.util.WaltLogger.IntLogger;
import frc.util.WaltLogger.StringLogger;

public class Superstructure {
    private final Coral m_coral;
    private final Elevator m_ele;

    private final DoubleConsumer m_driverRumbler, m_manipRumbler;

    private State m_state;

    // reqs
    private boolean autonIntakeReq = false;
    private boolean autonScoreEleReq = false;
    private boolean autonScoreReq = false;

    private boolean teleopCanScoreReq = false;

    private boolean preloadOverride = false;
    private boolean autonIntakeOverride = false;
    private boolean autonScoreOverride = false;

    public final EventLoop stateEventLoop = new EventLoop();

    private final Trigger trg_autonIntakeReq = new Trigger(() -> autonIntakeReq);
    private final Trigger trg_teleopIntakeReq;
    private final Trigger trg_botSensor;
    private final Trigger trg_autonScoreEleReq = new Trigger(() -> autonScoreEleReq);
    private final Trigger trg_teleopScoreEleReq;
    private final Trigger trg_autonScoreReq = new Trigger(() -> autonScoreReq);
    private final Trigger trg_teleopScoreReq;

    private final Trigger trg_eleNearSetpoint;

    private final Trigger trg_teleopIntakeEleOverride;
    private final Trigger trg_teleopIntakeOverride;
    private final Trigger trg_teleopScoreOverride;
    private final Trigger trg_teleopScoreEleOverride;
    private final Trigger trg_toHomeOverride;

    private final Trigger trg_preloadOverride = new Trigger(() -> preloadOverride);
    private final Trigger trg_autonIntakeOverride = new Trigger(() -> autonIntakeOverride);
    private final Trigger trg_autonScoreOverride = new Trigger(() -> autonScoreOverride);

    public final Trigger stateTrg_idle = new Trigger(stateEventLoop, () -> m_state == State.IDLE);
    public final Trigger stateTrg_eleToIntake = new Trigger(stateEventLoop, () -> m_state == State.ELE_TO_INTAKE);
    public final Trigger stateTrg_intaking = new Trigger(stateEventLoop, () -> m_state == State.INTAKING);
    public final Trigger stateTrg_intook = new Trigger(stateEventLoop, () -> m_state == State.INTOOK);
    public final Trigger stateTrg_eleToScore = new Trigger(stateEventLoop, () -> m_state == State.ELE_TO_SCORE);
    public final Trigger stateTrg_scoreReady = new Trigger(stateEventLoop, () -> m_state == State.SCORE_READY);
    public final Trigger stateTrg_score = new Trigger(stateEventLoop, () -> m_state == State.SCORING);
    public final Trigger stateTrg_scored = new Trigger(stateEventLoop, () -> m_state == State.SCORED);

    private EleHeight m_curHeightReq = EleHeight.HOME;

    private IntLogger log_state = WaltLogger.logInt(kLogTab, "state");

    public Superstructure(
        Coral coral, Elevator ele, 
        Trigger teleopIntakeReq,
        Trigger teleopEleHeightReq, 
        Trigger teleopScoreReq, 
        Trigger teleopIntakeEleOverride,
        Trigger teleopIntakeOverride,
        Trigger teleopScoreEleOverride,
        Trigger teleopScoreOverride,
        Trigger toHomeOverride,
        DoubleConsumer drivRumble, DoubleConsumer manipRumble
    ) {
        m_coral = coral;
        m_ele = ele;

        m_driverRumbler = drivRumble;
        m_manipRumbler = manipRumble;

        trg_teleopIntakeReq = teleopIntakeReq;
        trg_botSensor = new Trigger(m_coral.bs_botBeamBreak);
        trg_teleopScoreEleReq = teleopEleHeightReq;
        trg_teleopScoreReq = new Trigger(() -> teleopScoreReq.getAsBoolean() && teleopCanScoreReq);
        trg_eleNearSetpoint = new Trigger(() -> m_ele.nearSetpoint());

        trg_teleopIntakeEleOverride = teleopIntakeEleOverride;
        trg_teleopIntakeOverride = teleopIntakeOverride;
        trg_teleopScoreOverride = teleopScoreOverride;
        trg_teleopScoreEleOverride = teleopScoreEleOverride;
        trg_toHomeOverride = toHomeOverride;
        
        m_state = State.IDLE;

        configureAutonTrgs();
        configureTeleopTrgs();
        configureStateTransitions();
        configureStateActions();
    }

    private void configureAutonTrgs() {
        // these are treated kinda differently in auton than in teleop so i differentiated them again
        (stateTrg_idle.and(trg_autonIntakeReq))
            .onTrue(Commands.runOnce(() -> m_state = State.ELE_TO_INTAKE)); 
        (stateTrg_intook.and(trg_autonScoreEleReq))
            .onTrue(Commands.runOnce(() -> m_state = State.ELE_TO_SCORE));
        (stateTrg_scoreReady.and(trg_autonScoreReq))
            .onTrue(Commands.runOnce(() -> m_state = State.SCORING));
        
        // overrides
        (trg_preloadOverride.and(stateTrg_idle)).onTrue(Commands.runOnce(() -> m_state = State.INTOOK));
        (trg_autonIntakeOverride).onTrue(Commands.runOnce(() -> m_state = State.INTAKING));
        (trg_autonScoreOverride).onTrue(Commands.runOnce(() -> m_state = State.SCORING));
    }

    private void configureTeleopTrgs() {
        (stateTrg_idle.and(trg_teleopIntakeReq))
            .onTrue(Commands.runOnce(() -> m_state = State.ELE_TO_INTAKE));
        (stateTrg_intook.and(trg_teleopScoreEleReq))
            .onTrue(Commands.runOnce(() -> m_state = State.ELE_TO_SCORE));
        (stateTrg_scoreReady.and(trg_teleopScoreReq))
            .onTrue(Commands.runOnce(() -> m_state = State.SCORING));

        // overrides
        (trg_toHomeOverride).onTrue(Commands.runOnce(() -> m_state = State.IDLE));
        (trg_teleopIntakeEleOverride).onTrue(Commands.runOnce(() -> m_state = State.ELE_TO_INTAKE));
        (trg_teleopIntakeOverride).onTrue(Commands.runOnce(() -> m_state = State.INTAKING));
        (trg_teleopScoreEleOverride).onTrue(Commands.runOnce(() -> m_state = State.ELE_TO_SCORE));
        (trg_teleopScoreOverride).onTrue(Commands.runOnce(() -> m_state = State.SCORING));
    }

    private void configureStateTransitions() {
        (stateTrg_eleToIntake.and(trg_eleNearSetpoint))
            .onTrue(Commands.runOnce(() -> m_state = State.INTAKING));
        (stateTrg_intaking.and(trg_botSensor))
            .onTrue(Commands.runOnce(() -> m_state = State.INTOOK));
        (stateTrg_eleToScore.and(trg_eleNearSetpoint))
            .onTrue(Commands.runOnce(() -> m_state = State.SCORE_READY));
        (stateTrg_score.and(trg_botSensor.negate()))
            .onTrue(Commands.runOnce(() -> m_state = State.SCORED));
        (stateTrg_scored.and(trg_toHomeOverride.negate()))
            .onTrue(Commands.runOnce(() -> m_state = State.ELE_TO_INTAKE));
    }

    private void configureStateActions() {
        (stateTrg_idle)
            .onTrue(m_ele.toPosition(EleHeight.HOME));
        (stateTrg_eleToIntake)
            .onTrue(m_ele.toPosition(EleHeight.HP));
        (stateTrg_intaking)
            .onTrue(m_coral.setCoralMotorAction(kCoralSpeed));
        (stateTrg_intook)
            .onTrue(m_coral.setCoralMotorAction(0)
            .andThen(manipRumble(kRumbleIntensity, kRumbleTimeoutSecs)));
        (stateTrg_eleToScore)
            .onTrue(m_ele.toPosition(m_curHeightReq));
        (stateTrg_scoreReady)
            .onTrue(Commands.runOnce(() -> teleopCanScoreReq = true)
            .andThen(driverRumble(kRumbleIntensity, kRumbleTimeoutSecs)));
        (stateTrg_score)
            .onTrue(m_coral.setCoralMotorAction(kCoralSpeed));
        (stateTrg_scored)
            .onTrue(m_coral.setCoralMotorAction(0)
            .alongWith(Commands.runOnce(() -> teleopCanScoreReq = false)));
    }

    private Command driverRumble(double intensity, double secs) {
        return Commands.run(
           () -> m_driverRumbler.accept(0)
        ).withTimeout(secs);
    }

    private Command manipRumble(double intensity, double secs) {
        return Commands.run(
            () -> m_manipRumbler.accept(0)
        ).withTimeout(secs);
    }

    public void requestIsPreload(boolean preload) {
        preloadOverride = preload;
    }

    public Command autonRequestEleToScore(EleHeight height) {
        return requestEleToScore(height).alongWith(Commands.runOnce(() -> autonScoreEleReq = true));
    }

    public Command requestEleToScore(EleHeight height) {
        return Commands.runOnce(() -> m_curHeightReq = height);
    }

    public Command autonScoreReq() {
        return Commands.runOnce(() -> autonScoreReq = true);
    }

    public Command autonRequestToIntake() {
        return requestToIntake().alongWith(Commands.runOnce(() -> autonIntakeReq = true));
    }

    public Command requestToIntake() {
        return Commands.runOnce(() -> m_curHeightReq = EleHeight.HP);
    }

    public void logState() {
        log_state.accept(m_state.idx);
    }

    public enum State {
        IDLE(0),
        ELE_TO_INTAKE(1),
        INTAKING(2),
        INTOOK(3),
        ELE_TO_SCORE(4),
        SCORE_READY(5),
        SCORING(6),
        SCORED(7);

        public final int idx;
  
        private State(int index) {
            idx = index;
        }
    }
}
