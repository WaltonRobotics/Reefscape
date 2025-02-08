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
    private final Coral m_coral;
    private final Elevator m_ele;

    private final DoubleConsumer m_driverRumbler, m_manipRumbler;

    private State m_state;

    // reqs
    private boolean autonIntakeReq = false;
    private boolean autonScoreEleReq = false;
    private boolean autonScoreReq = false;

    private boolean driverRumbled = false;
    private boolean manipRumbled = false;

    // overrides
    private boolean teleopIntakeEleOverride = false; // manip only
    private boolean teleopIntakeOverride = false; // manip only
    private boolean teleopScoreOverride = false; // drive only
    private boolean teleopScoreEleOverride = false; // manip only
    private boolean toHomeReq = false; // manip only

    private boolean preloadOverride = false;
    private boolean autonIntakeOverride = false;
    private boolean autonScoreOverride = false;

    public final EventLoop stateEventLoop = new EventLoop();

    public final Trigger trg_autonIntakeReq = new Trigger(() -> autonIntakeReq);
    public final Trigger trg_teleopIntakeReq;
    public final Trigger trg_eleAtIntake;
    public final Trigger trg_botSensor;
    public final Trigger trg_autonScoreEleReq = new Trigger(() -> autonScoreEleReq);
    public final Trigger trg_teleopScoreEleReq;
    public final Trigger trg_eleAtScore;
    public final Trigger trg_autonScoreReq = new Trigger(() -> autonScoreReq);
    public final Trigger trg_teleopScoreReq;
    public final Trigger trg_botSensorFalsed;

    public final Trigger trg_driverRumbled = new Trigger(() -> driverRumbled);
    public final Trigger trg_manipRumbled = new Trigger(() -> manipRumbled);

    public final Trigger trg_teleopIntakeEleOverride = new Trigger(() -> teleopIntakeEleOverride);
    public final Trigger trg_teleopIntakeOverride = new Trigger(() -> teleopIntakeOverride);
    public final Trigger trg_teleopScoreOverride = new Trigger(() -> teleopScoreOverride);
    public final Trigger trg_teleopScoreEleOverride = new Trigger(() -> teleopScoreEleOverride);
    public final Trigger trg_toHomeOverride = new Trigger(() -> toHomeReq);

    public final Trigger trg_preloadOverride = new Trigger(() -> preloadOverride);
    public final Trigger trg_autonIntakeOverride = new Trigger(() -> autonIntakeOverride);
    public final Trigger trg_autonScoreOverride = new Trigger(() -> autonScoreOverride);

    public final Trigger stateTrg_idle = new Trigger(stateEventLoop, () -> m_state == State.IDLE);
    public final Trigger stateTrg_eleToIntake = new Trigger(stateEventLoop, () -> m_state == State.ELE_TO_INTAKE);
    public final Trigger stateTrg_intaking = new Trigger(stateEventLoop, () -> m_state == State.INTAKING);
    public final Trigger stateTrg_intook = new Trigger(stateEventLoop, () -> m_state == State.INTOOK);
    public final Trigger stateTrg_eleToScore = new Trigger(stateEventLoop, () -> m_state == State.ELE_TO_SCORE);
    public final Trigger stateTrg_scoreReady = new Trigger(stateEventLoop, () -> m_state == State.SCORE_READY);
    public final Trigger stateTrg_score = new Trigger(stateEventLoop, () -> m_state == State.SCORING);
    public final Trigger stateTrg_scored = new Trigger(stateEventLoop, () -> m_state == State.SCORED);

    public Superstructure(Coral coral, Elevator ele, Trigger teleopIntakeReq, Trigger teleopEleHeightReq, Trigger teleopScoreReq, DoubleConsumer drivRumble, DoubleConsumer manipRumble) {
        m_coral = coral;
        m_ele = ele;

        m_driverRumbler = drivRumble;
        m_manipRumbler = manipRumble;

        trg_teleopIntakeReq = teleopIntakeReq;
        trg_eleAtIntake = new Trigger(m_ele.bs_atIntakeHeight);
        trg_botSensor = new Trigger(m_coral.bs_botBeamBreak);
        trg_teleopScoreEleReq = teleopEleHeightReq;
        trg_eleAtScore = new Trigger(m_ele.bs_atScoreHeight);
        trg_teleopScoreReq = teleopScoreReq;
        trg_botSensorFalsed = new Trigger(() -> !m_coral.bs_botBeamBreak.getAsBoolean());
        
        m_state = State.IDLE;

        configureAutonTrgs();
        configureTeleopTrgs();
    }

    private void configureAutonTrgs() {
        (trg_autonIntakeReq.and(stateTrg_idle)).onTrue(Commands.runOnce(() -> m_state = State.ELE_TO_INTAKE));
        (trg_eleAtIntake.and(stateTrg_eleToIntake).and(RobotModeTriggers.autonomous())).onTrue(Commands.runOnce(() -> m_state = State.INTAKING));
        (trg_botSensor.and(stateTrg_intaking).and(RobotModeTriggers.autonomous())).onTrue(Commands.runOnce(() -> m_state = State.INTOOK));
        (trg_autonScoreEleReq.and(stateTrg_intook)).onTrue(Commands.runOnce(() -> m_state = State.ELE_TO_SCORE));
        (trg_eleAtScore.and(stateTrg_eleToScore).and(RobotModeTriggers.autonomous())).onTrue(Commands.runOnce(() -> m_state = State.SCORE_READY));
        (trg_autonScoreReq.and(stateTrg_scoreReady)).onTrue(Commands.runOnce(() -> m_state = State.SCORING));
        (trg_botSensorFalsed.and(stateTrg_score).and(RobotModeTriggers.autonomous())).onTrue(Commands.runOnce(() -> m_state = State.SCORED));
        (stateTrg_scored.and(stateTrg_scored).and(RobotModeTriggers.autonomous())).onTrue(Commands.runOnce(() -> m_state = State.ELE_TO_INTAKE));

        //overrides
        (trg_preloadOverride.and(stateTrg_idle)).onTrue(Commands.runOnce(() -> m_state = State.INTOOK));
        (trg_autonIntakeOverride).onTrue(Commands.runOnce(() -> m_state = State.INTAKING));
        (trg_autonScoreOverride).onTrue(Commands.runOnce(() -> m_state = State.SCORING));
    }

    private void configureTeleopTrgs() {
        (trg_teleopIntakeReq.and(stateTrg_idle)).onTrue(Commands.runOnce(() -> m_state = State.ELE_TO_INTAKE));
        (trg_eleAtIntake.and(stateTrg_eleToIntake).and(RobotModeTriggers.teleop())).onTrue(Commands.runOnce(() -> m_state = State.INTAKING));
        (trg_botSensor.and(stateTrg_intaking).and(RobotModeTriggers.teleop())).onTrue(Commands.runOnce(() -> m_state = State.INTOOK));
        (trg_teleopScoreEleReq.and(stateTrg_intook)).onTrue(Commands.runOnce(() -> m_state = State.ELE_TO_SCORE));
        (trg_eleAtScore.and(stateTrg_eleToScore).and(RobotModeTriggers.teleop())).onTrue(Commands.runOnce(() -> m_state = State.SCORE_READY));
        (trg_teleopScoreReq.and(stateTrg_scoreReady)).onTrue(Commands.runOnce(() -> m_state = State.SCORING));
        (trg_botSensorFalsed.and(stateTrg_score).and(RobotModeTriggers.teleop())).onTrue(Commands.runOnce(() -> m_state = State.SCORED));
        (stateTrg_scored.and(stateTrg_scored).and(RobotModeTriggers.teleop())).onTrue(Commands.runOnce(() -> m_state = State.ELE_TO_INTAKE));

        //overrides
        (trg_toHomeOverride).onTrue(Commands.runOnce(() -> m_state = State.IDLE));
        (trg_teleopIntakeEleOverride).onTrue(Commands.runOnce(() -> m_state = State.ELE_TO_INTAKE));
        (trg_teleopIntakeOverride).onTrue(Commands.runOnce(() -> m_state = State.INTAKING));
        (trg_teleopScoreEleOverride).onTrue(Commands.runOnce(() -> m_state = State.ELE_TO_SCORE));
        (trg_teleopScoreOverride).onTrue(Commands.runOnce(() -> m_state = State.SCORING));
    }

    private Command driverRumble(double intensity, double secs) {
        return Commands.startEnd(
            () -> {
                if(!driverRumbled) {
                    m_driverRumbler.accept(intensity);
                    driverRumbled = true;
                }
            },
            () -> m_driverRumbler.accept(0)
        ).withTimeout(secs);
    }

    private Command manipRumble(double intensity, double secs) {
        return Commands.startEnd(
            () -> {
                if(!manipRumbled) {
                    m_manipRumbler.accept(intensity);
                    manipRumbled = true;
                }
            },
            () -> m_manipRumbler.accept(0)
        ).withTimeout(secs);
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
