package frc.robot.subsystems;

import static frc.robot.Constants.RobotK.*;

import java.util.function.DoubleConsumer;

import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Elevator.EleHeights;
import frc.util.WaltLogger;
import frc.util.WaltLogger.IntLogger;

public class Superstructure {
    public Coral m_coral;
    public Elevator m_ele;

    private CoralState m_coralState;
    private boolean m_driverRumbled = false;

    private boolean m_preload = false;
    private boolean m_autonScoreReq = false;
    private boolean m_autonScoreOverride = false;
    private boolean m_autonEleIntakeReady = false;

    public final EventLoop stateEventLoop = new EventLoop();

    private final DoubleConsumer m_driverRumbler; // when SCORE_READY

    private final Trigger trg_preloadReq = new Trigger(() -> m_preload).and(RobotModeTriggers.autonomous());
    private final Trigger trg_driverScoreReq;
    private final Trigger trg_driverScoreOverrideReq;
    private final Trigger trg_autonScoreReq = new Trigger(() -> m_autonScoreReq).and(RobotModeTriggers.autonomous());
    private final Trigger trg_autonScoreOverride = new Trigger(() -> m_autonScoreOverride).and(RobotModeTriggers.autonomous());
    private final Trigger trg_autonEleIntakeReady = new Trigger(() -> m_autonEleIntakeReady).and(RobotModeTriggers.autonomous());

    /* NOTE: sensor trigs are in Coral.java */
    public final Trigger stateTrg_idle = new Trigger(stateEventLoop, () -> m_coralState.equals(CoralState.IDLE));
    public final Trigger stateTrg_intaking = new Trigger(stateEventLoop, () -> m_coralState.equals(CoralState.INTAKING));
    public final Trigger stateTrg_intook = new Trigger(stateEventLoop, () -> m_coralState.equals(CoralState.INTOOK));
    public final Trigger stateTrg_scoreReady = new Trigger(stateEventLoop, () -> m_coralState.equals(CoralState.SCORE_READY));
    public final Trigger stateTrg_scoring = new Trigger(stateEventLoop, () -> m_coralState.equals(CoralState.SCORING));
    public final Trigger stateTrg_autonIntakeReady = new Trigger(stateEventLoop, trg_autonEleIntakeReady);

    private final IntLogger log_state = WaltLogger.logInt(kLogTab, "state", PubSubOption.sendAll(true));
    
    public Superstructure(Algae algae, Coral coral, Elevator ele, DoubleConsumer driverRumbler, Trigger score, Trigger scoreOverride) {
        m_coral = coral;
        m_ele = ele; 

        m_driverRumbler = driverRumbler;
        trg_driverScoreReq = score;
        trg_driverScoreOverrideReq = scoreOverride;
        m_coralState = CoralState.IDLE;

        log_state.accept(m_coralState.m_index);

        configStateTrigs();
    }

    private void configStateTrigs() {
        // stateTrg_idle
        if(RobotModeTriggers.autonomous().getAsBoolean()) {
            (stateTrg_scoring.and(() -> !m_coral.bs_coralMotorRunning.getAsBoolean()))
                .onTrue(Commands.runOnce(() -> m_coralState = CoralState.IDLE).andThen(Commands.runOnce(() -> m_autonScoreReq = false)));
        } else {
            (stateTrg_scoring.and(() -> !m_coral.bs_coralMotorRunning.getAsBoolean())).onTrue(Commands.runOnce(() -> m_coralState = CoralState.IDLE));
        }

        // stateTrg_autonIntakeReady
        (RobotModeTriggers.autonomous().and(stateTrg_idle).and(trg_autonEleIntakeReady)).onTrue(Commands.runOnce(() -> m_coralState = CoralState.INTAKING));

        // stateTrg_intaking
        (RobotModeTriggers.teleop().and(stateTrg_idle).and(m_coral.bs_topBeamBreak).and(() -> !m_coral.bs_botBeamBreak.getAsBoolean()))
            .onTrue(Commands.runOnce(() -> m_coralState = CoralState.INTAKING));
        stateTrg_intaking.onTrue(m_coral.intake());
    
        // stateTrg_intook
        (stateTrg_intaking.and(() -> !m_coral.bs_botBeamBreak.getAsBoolean())).onTrue(Commands.runOnce(() -> m_coralState = CoralState.INTOOK)
            .alongWith(Commands.runOnce(() -> {
                if(RobotModeTriggers.autonomous().getAsBoolean()) {
                    m_autonEleIntakeReady = false;
                }
            })));

        // stateTrg_scoreReady
        ((stateTrg_intook.and(m_ele.bs_atHeight)))
            .onTrue(Commands.runOnce(() -> m_coralState = CoralState.SCORE_READY));
        (RobotModeTriggers.autonomous().and(stateTrg_scoreReady)).onTrue(Commands.runOnce(() -> m_preload = false));
        (RobotModeTriggers.teleop().and(stateTrg_scoreReady)).onTrue(driverRumble(1, 0.5));

        // stateTrg_scoring
        (stateTrg_scoreReady.and(trg_driverScoreReq)).onTrue(Commands.runOnce(() -> m_coralState = CoralState.SCORING));
        (stateTrg_scoreReady.and(trg_autonScoreReq)).onTrue(Commands.runOnce(() -> m_coralState = CoralState.SCORING));
        stateTrg_scoring.onTrue(m_coral.score());
        (RobotModeTriggers.autonomous().and(stateTrg_scoring).and(trg_autonScoreOverride)).onTrue(Commands.runOnce(() -> m_autonScoreOverride = false));

        // preload intake bypass
        (stateTrg_idle.and(() -> trg_preloadReq.getAsBoolean())).onTrue(Commands.runOnce(() -> m_coralState = CoralState.INTOOK));

        // score override
        trg_driverScoreOverrideReq.onTrue(Commands.runOnce(() -> m_coralState = CoralState.SCORING));
        trg_autonScoreOverride.onTrue(Commands.runOnce(() -> m_coralState = CoralState.SCORING));
    }

    public void resetAfterAuton() {
        m_preload = false;
        m_autonScoreReq = false;
        m_autonScoreOverride = false;
    }

    public void autonPreload() {
        m_preload = true;
    }

    public void autonScore() {
        m_autonScoreReq = true;
    }

    /*
     * use for scoring purposes ONLY. dont try ot pass in hp height PLS or else ill CRY
     */
    public Command autonToHeightCmd(EleHeights height) {
        return m_ele.setPosition(height).andThen(Commands.runOnce(() -> m_coralState = CoralState.SCORE_READY));
    }

    public Command autonToIntakeHeightCmd() {
        return m_ele.setPosition(EleHeights.HP).andThen(Commands.runOnce(() -> m_autonEleIntakeReady = true));
    }

    public void autonScoreOverrideReq() {
        m_autonScoreOverride = true;
    }

    private Command driverRumble(double intensity, double secs) {
        return Commands.startEnd(
            () -> {
                if(!m_driverRumbled) {
                    m_driverRumbler.accept(intensity);
                    m_driverRumbled = true;
                }
            },
            () -> m_driverRumbler.accept(0)
        ).withTimeout(secs);
    }

    public CoralState getCoralState() {
        return m_coralState;
    }

    public enum CoralState {
        IDLE(0), // nothing's in there (like my brain)
        INTAKING(1), // wheels are running to secure the coral. runs between topBeamBreak breaking and botBeamBreak breaking
        INTOOK(2), // coral is secure, but ele isn't in place
        SCORE_READY(3), // ele at the right scoring height
        SCORING(4); // wheels are running to expel the coral. returns to idle when botBeamBreak unbreaks

        public int m_index;

        private CoralState(int index) {
            m_index = index;
        }
    }
}
