package frc.robot.subsystems;

import static frc.robot.Constants.RobotK.*;

import java.util.function.DoubleConsumer;

import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.util.WaltLogger;
import frc.util.WaltLogger.IntLogger;

public class Superstructure {
    public Coral m_coral;
    public Elevator m_ele;

    private CoralState m_coralState;
    private boolean m_driverRumbled = false;

    private boolean m_preload = false;

    public final EventLoop stateEventLoop = new EventLoop();

    public final Timer m_autonTimer = new Timer();

    private final DoubleConsumer m_driverRumbler; // when SCORE_READY

    private final Trigger trg_preloadReq = new Trigger(() -> m_preload).and(RobotModeTriggers.autonomous());
    private final Trigger trg_driverScoreReq;

    /* NOTE: sensor trigs are in Coral.java */
    public final Trigger stateTrg_idle = new Trigger(stateEventLoop, () -> m_coralState.equals(CoralState.IDLE));
    public final Trigger stateTrg_intaking = new Trigger(stateEventLoop, () -> m_coralState.equals(CoralState.INTAKING));
    public final Trigger stateTrg_intook = new Trigger(stateEventLoop, () -> m_coralState.equals(CoralState.INTOOK));
    public final Trigger stateTrg_scoreReady = new Trigger(stateEventLoop, () -> m_coralState.equals(CoralState.SCORE_READY));
    public final Trigger stateTrg_scoring = new Trigger(stateEventLoop, () -> m_coralState.equals(CoralState.SCORING));

    private final IntLogger log_state = WaltLogger.logInt(kLogTab, "state", PubSubOption.sendAll(true));
    
    public Superstructure(Algae algae, Coral coral, Elevator ele, DoubleConsumer driverRumbler, Trigger shooting) {
        m_coral = coral;
        m_ele = ele; 

        m_driverRumbler = driverRumbler;
        trg_driverScoreReq = shooting;
        m_coralState = CoralState.IDLE;

        log_state.accept(m_coralState.m_index);

        configStateTrigs();
    }

    private void configStateTrigs() {
        // stateTrg_idle
        (stateTrg_scoring.and(() -> !m_coral.bs_coralMotorRunning.getAsBoolean())).onTrue(Commands.runOnce(() -> m_coralState = CoralState.IDLE));

        // stateTrg_intaking
        ((stateTrg_idle.or(trg_preloadReq)).and(m_coral.bs_topBeamBreak).and(() -> !m_coral.bs_botBeamBreak.getAsBoolean()))
            .onTrue(Commands.runOnce(() -> m_coralState = CoralState.INTAKING));
        stateTrg_intaking.onTrue(m_coral.intake());
    
        // stateTrg_intook
        (stateTrg_intaking.and(() -> !m_coral.bs_botBeamBreak.getAsBoolean())).onTrue(Commands.runOnce(() -> m_coralState = CoralState.INTOOK));

        // stateTrg_scoreReady
        ((stateTrg_intook.and(m_ele.bs_atHeight))).onTrue(Commands.runOnce(() -> m_coralState = CoralState.SCORE_READY));
        stateTrg_intook.onTrue(DriverRumble(1, 0.5));

        // stateTrg_scoring
        (stateTrg_scoreReady.and(trg_driverScoreReq)).onTrue(Commands.runOnce(() -> m_coralState = CoralState.SCORING));
        stateTrg_scoring.onTrue(m_coral.score().andThen(() -> m_ele.invertAtHeight()));

        // bypass intaking for autons
        (stateTrg_idle.and(() -> trg_preloadReq.getAsBoolean())).onTrue(Commands.runOnce(() -> m_coralState = CoralState.INTOOK));
    }

    private Command DriverRumble(double intensity, double secs) {
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

    private Command changeState(CoralState newState) {
        return Commands.runOnce(() -> {
            if(m_coralState.equals(newState)) {
                return;
            }
            CoralState prevState = m_coralState;
            m_coralState = newState;
            Commands.print("state change from " + prevState + " to " + m_coralState);
        }).withName("stateChange_to" + newState);
    }

    public Command forceStateChange(CoralState state) {
        return Commands.parallel(
            changeState(state)
        );
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
