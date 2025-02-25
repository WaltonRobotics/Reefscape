package frc.robot.subsystems;

import static frc.robot.Constants.kRumbleIntensity;
import static frc.robot.Constants.kRumbleTimeoutSecs;
import static frc.robot.Constants.RobotK.*;

import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;

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
    private boolean m_eleToL1Req = false;
    private boolean m_eleToL2Req = false;
    private boolean m_eleToL3Req = false;
    private boolean m_eleToL4Req = false;
    private boolean m_scoreReq = false;

    private boolean m_simIntook = false;
    private boolean m_simScored = false;

    /* state transitions */
    /* autoTrgs */
    /* teleopTrgs */
    private final Trigger trg_teleopEleToHPReq;
    private final Trigger trg_teleopL1Req; 
    private final Trigger trg_teleopL2Req; 
    private final Trigger trg_teleopL3Req; 
    private final Trigger trg_teleopL4Req; 
    private final Trigger trg_teleopScoreReq;
    /* teleopTrgs: overrides */
    private final Trigger transTrg_toIdleOverrideReq;
    /* sim transitions */
    private final Trigger simTransTrg_intook = new Trigger(() -> m_simIntook);
    private final Trigger simTransTrg_scored = new Trigger(() -> m_simScored);
    /* Frsies Transition Trigs */
    private final Trigger transTrg_eleToHP = new Trigger(() -> m_eleToHPStateTransReq);
    private final Trigger transTrg_eleNearSetpt; // used for any ele mvmt state
    private final Trigger transTrg_topSensor;
    private final Trigger transTrg_botSensor;
    private final Trigger transTrg_eleToL1 = new Trigger(() -> m_eleToL1Req);
    private final Trigger transTrg_eleToL2 = new Trigger(() -> m_eleToL2Req);
    private final Trigger transTrg_eleToL3 = new Trigger(() -> m_eleToL3Req);
    private final Trigger transTrg_eleToL4 = new Trigger(() -> m_eleToL4Req);
    private final Trigger transTrg_scoring = new Trigger(() -> m_scoreReq);

    /* states */
    public final Trigger stateTrg_idle = new Trigger(stateEventLoop, () -> m_state == State.IDLE);
    public final Trigger stateTrg_eleToHP = new Trigger(stateEventLoop, () -> m_state == State.ELE_TO_HP);
    public final Trigger stateTrg_intaking = new Trigger(stateEventLoop, () -> m_state == State.INTAKING);
    public final Trigger stateTrg_intook = new Trigger(stateEventLoop, () -> m_state == State.INTOOK);
    public final Trigger stateTrg_eleToL1 = new Trigger(stateEventLoop, () -> m_state == State.ELE_TO_L1);
    public final Trigger stateTrg_eleToL2 = new Trigger(stateEventLoop, () -> m_state == State.ELE_TO_L2);
    public final Trigger stateTrg_eleToL3 = new Trigger(stateEventLoop, () -> m_state == State.ELE_TO_L3);
    public final Trigger stateTrg_eleToL4 = new Trigger(stateEventLoop, () -> m_state == State.ELE_TO_L4);
    public final Trigger stateTrg_scoreReady = new Trigger(stateEventLoop, () -> m_state == State.SCORE_READY);
    public final Trigger stateTrg_scoring = new Trigger(stateEventLoop, () -> m_state == State.SCORING);
    public final Trigger stateTrg_scored = new Trigger(stateEventLoop, () -> m_state == State.SCORED);

    public final Trigger stateTrg_eleToClimb = new Trigger(stateEventLoop, () -> m_state == State.ELE_TO_CLIMB);
    public final Trigger stateTrg_climbReady = new Trigger(stateEventLoop, () -> m_state == State.CLIMB_READY);
    public final Trigger stateTrg_climbing = new Trigger(stateEventLoop, () -> m_state == State.CLIMBING);
    public final Trigger stateTrg_climbed = new Trigger(stateEventLoop, () -> m_state == State.CLIMBED);

    /* sm odds & ends */
    private final DoubleConsumer m_driverRumbler;

    /* loggin' */
    private DoubleLogger log_stateIdx = WaltLogger.logDouble(kLogTab, "state idx");
    private StringLogger log_stateName = WaltLogger.logString(kLogTab, "state name");
    
    /* logs: state trans */
    private BooleanLogger log_teleopToHPReq = WaltLogger.logBoolean(kLogTab, "TELEOP to HP req");
    private BooleanLogger log_teleopScoreReq = WaltLogger.logBoolean(kLogTab, "TELEOP score req");
    private BooleanLogger log_toIdleOverride = WaltLogger.logBoolean(kLogTab, "to idle override");

    private BooleanLogger log_eleToHPReq = WaltLogger.logBoolean(kLogTab, "ele to HP req");
    private BooleanLogger log_eleAtSetpt = WaltLogger.logBoolean(kLogTab, "ele at setpoint");
    private BooleanLogger log_topSensor = WaltLogger.logBoolean(kLogTab, "top beam break");
    private BooleanLogger log_botSensor = WaltLogger.logBoolean(kLogTab, "bot beam break");
    private BooleanLogger log_eleToL1Req = WaltLogger.logBoolean(kLogTab, "ele to lvl 1 req");
    private BooleanLogger log_eleToL2Req = WaltLogger.logBoolean(kLogTab, "ele to lvl 2 req");
    private BooleanLogger log_eleToL3Req = WaltLogger.logBoolean(kLogTab, "ele to lvl 3 req");
    private BooleanLogger log_eleToL4Req = WaltLogger.logBoolean(kLogTab, "ele to lvl 4 req");
    private BooleanLogger log_scoringReq = WaltLogger.logBoolean(kLogTab, "score req");
    /* sim stuff */
    private BooleanLogger log_simIntook = WaltLogger.logBoolean(kLogTab, "SIM intook");
    private BooleanLogger log_simScored = WaltLogger.logBoolean(kLogTab, "SIM scored");

    public Superstructure(
        Coral coral,
        Elevator ele,
        Trigger eleToHPReq,
        Trigger L1Req,
        Trigger L2Req,
        Trigger L3Req,
        Trigger L4Req,
        Trigger scoreReq,
        Trigger toIdleOverride,
        DoubleConsumer driverRumbler
    ) {
        m_coral = coral;
        m_ele = ele;

        /* teleop trigs */
        trg_teleopEleToHPReq = eleToHPReq;
        trg_teleopL1Req = L1Req;
        trg_teleopL2Req = L2Req;
        trg_teleopL3Req = L3Req;
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
        configureSimTransitions();
        configureStateActions();
    }

    /* configs */
    private void configureRequests() {
        (trg_teleopEleToHPReq.and(RobotModeTriggers.teleop()))
            .onTrue(Commands.runOnce(() -> m_eleToHPStateTransReq = true));
        (trg_teleopL1Req.and(RobotModeTriggers.teleop()))
            .onTrue(Commands.runOnce(() -> m_eleToL1Req = true));
        (trg_teleopL2Req.and(RobotModeTriggers.teleop()))
            .onTrue(Commands.runOnce(() -> m_eleToL2Req = true));
        (trg_teleopL3Req.and(RobotModeTriggers.teleop()))
            .onTrue(Commands.runOnce(() -> m_eleToL3Req = true));
        (trg_teleopL4Req.and(RobotModeTriggers.teleop()))
            .onTrue(Commands.runOnce(() -> m_eleToL4Req = true));
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
        (stateTrg_intook.and(transTrg_eleToL4))
            .onTrue(Commands.runOnce(() -> m_state = State.ELE_TO_L4)
            .alongWith(Commands.print("TESTING 2")));
        /* TODO: make debouncer time faster */
        (stateTrg_eleToL1.debounce(1).and(transTrg_eleNearSetpt))
            .onTrue(Commands.runOnce(() -> m_state = State.SCORE_READY)); 
        (stateTrg_eleToL2.debounce(1).and(transTrg_eleNearSetpt))
            .onTrue(Commands.runOnce(() -> m_state = State.SCORE_READY)); 
        (stateTrg_eleToL3.debounce(1).and(transTrg_eleNearSetpt))
            .onTrue(Commands.runOnce(() -> m_state = State.SCORE_READY)); 
        (stateTrg_eleToL4.debounce(1).and(transTrg_eleNearSetpt))
            .onTrue(Commands.runOnce(() -> m_state = State.SCORE_READY)); 
        (stateTrg_scoreReady.and(transTrg_scoring)) 
            .onTrue(Commands.runOnce(() -> m_state = State.SCORING));
        (stateTrg_scoring.and(transTrg_botSensor.negate())) 
            .onTrue(Commands.runOnce(() -> m_state = State.SCORED));
        (stateTrg_scored.debounce(0.02))
            .onTrue(Commands.runOnce(() -> m_state = State.ELE_TO_HP));


        (transTrg_toIdleOverrideReq)
            .onTrue(Commands.runOnce(() -> m_state = State.IDLE));
    }

    // cuz i dont have a joystick myself and ill usually use sim at home, im going to automate everything
    private void configureSimTransitions() {
        (stateTrg_idle.and(() -> Utils.isSimulation()).and(RobotModeTriggers.teleop())).debounce(1) 
            .onTrue(
                Commands.sequence(
                    Commands.print("SIM TO ELE TO HP"),
                    Commands.runOnce(() -> m_eleToHPStateTransReq = true)
                )
            );
        (stateTrg_intaking.and(() -> Utils.isSimulation()).and(RobotModeTriggers.teleop())).debounce(0.5)
            .onTrue(simIntook());
        (stateTrg_intook.and(() -> Utils.isSimulation())).debounce(1)
            .onTrue(
                Commands.sequence(
                    Commands.runOnce(() -> m_eleToL4Req = true)
                )
            );
        (stateTrg_scoring.and(() -> Utils.isSimulation()).and(RobotModeTriggers.teleop())).debounce(0.5)
            .onTrue(simScored());

        simTransTrg_intook
            .onTrue(Commands.runOnce(() -> m_state = State.INTOOK));
        simTransTrg_scored
            .onTrue(Commands.runOnce(() -> m_state = State.SCORED));
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
        
        stateTrg_eleToL1
            .onTrue(
                Commands.parallel(
                        m_ele.toHeight(() -> L1),
                        Commands.runOnce(() -> m_eleToL1Req = false)
                )
            );

        stateTrg_eleToL2
            .onTrue(
                Commands.parallel(
                        m_ele.toHeight(() -> L2),
                        Commands.runOnce(() -> m_eleToL2Req = false)
                )
            );

        stateTrg_eleToL3
            .onTrue(
                Commands.parallel(
                        m_ele.toHeight(() -> L3),
                        Commands.runOnce(() -> m_eleToL3Req = false)
                )
            );

        stateTrg_eleToL4
            .onTrue(
                Commands.parallel(
                        m_ele.toHeight(() -> L4),
                        Commands.runOnce(() -> m_eleToL4Req = false)
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

    /* to be used in auton */

    /* to be used in sim */
    public Command simIntook() {
        return Commands.runOnce(() -> m_simIntook = true);
    }

    public Command simScored() {
        return Commands.runOnce(() -> m_simScored = true);
    }

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
        log_eleToL1Req.accept(transTrg_eleToL1);
        log_eleToL2Req.accept(transTrg_eleToL2);
        log_eleToL3Req.accept(transTrg_eleToL3);
        log_eleToL4Req.accept(transTrg_eleToL4);
        log_scoringReq.accept(transTrg_scoring);

        log_toIdleOverride.accept(transTrg_toIdleOverrideReq);
    }

    public void logSimThings() {
        log_simIntook.accept(m_simIntook);
        log_simScored.accept(m_simScored);
    }

    public void periodic() {
        stateEventLoop.poll();
        logRequests();
        logStateChangeReqs();
        logState();

        if(Utils.isSimulation()) {
            logSimThings();
        }
    }

    public enum State {
        IDLE(0, "idle"),
        ELE_TO_HP(1, "ele to intake"),
        INTAKING(2, "intaking"),
        INTOOK(3, "intook"),
        ELE_TO_L1(4.1, "ele to L1"),
        ELE_TO_L2(4.2, "ele to L2"),
        ELE_TO_L3(4.3, "ele to L3"),
        ELE_TO_L4(4.4, "ele to L4"),
        SCORE_READY(5, "score ready"),
        SCORING(6, "scoring"),
        SCORED(7, "scored"),

        ELE_TO_CLIMB(8, "ele to climb"),
        CLIMB_READY(9, "climb ready"),
        CLIMBING(10, "climbing"),
        CLIMBED(11, "climbed");

        public final double idx;
        public final String name;
  
        private State(double index, String _name) {
            idx = index;
            name = _name;
        }
    }
}
