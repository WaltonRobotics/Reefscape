package frc.robot.subsystems;

import static frc.robot.Constants.kRumbleIntensity;
import static frc.robot.Constants.kRumbleTimeoutSecs;
import static frc.robot.Constants.RobotK.*;

import java.util.Optional;
import java.util.function.DoubleConsumer;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.PubSubOptions;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.subsystems.Elevator.EleHeight.*;

import frc.robot.Robot;
import frc.robot.subsystems.Elevator.AlgaeHeight;
import frc.robot.subsystems.Elevator.EleHeight;
import frc.robot.vision.Vision;
import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;
import frc.util.WaltLogger.IntLogger;
import frc.util.WaltLogger.StringLogger;

public class Superstructure {
    private final Coral m_coral;
    private final Finger m_finger;
    private final Elevator m_ele;
    private final Optional<Vision> m_cam1;
    private final Funnel m_funnel;

    public final EventLoop stateEventLoop = new EventLoop();
    public State m_state = State.IDLE;

    private final Trigger trg_isSimulation = new Trigger(Robot::isSimulation);

    /* requests */
    private boolean m_l1Toggle = false;
    /* reqs: auton */
    private boolean m_autonEleToHPReq = false;
    private boolean m_autonEleToL1Req = false;
    private boolean m_autonEleToL2Req = false;
    private boolean m_autonEleToL3Req = false;
    private boolean m_autonEleToL4Req = false;
    private boolean m_autonScoreReq = false;

    private boolean m_simIntook = false;
    private boolean m_simScored = false;
    private boolean m_simHasCoral = false;

    /* state transitions */
    private final Trigger trg_l1Toggle = new Trigger(() -> m_l1Toggle);
    /* autoTrgs */
    private final Trigger trg_autonEleToHPReq = new Trigger(stateEventLoop, () -> m_autonEleToHPReq);
    private final Trigger trg_autonL1Req = new Trigger(stateEventLoop, () -> m_autonEleToL1Req); 
    private final Trigger trg_autonL2Req = new Trigger(stateEventLoop, () -> m_autonEleToL2Req); 
    private final Trigger trg_autonL3Req = new Trigger(stateEventLoop, () -> m_autonEleToL3Req); 
    private final Trigger trg_autonL4Req = new Trigger(stateEventLoop, () -> m_autonEleToL4Req); 
    private final Trigger trg_autonScoreReq = new Trigger(stateEventLoop, () -> m_autonScoreReq);
    /* teleopTrgs */
    private final Trigger trg_teleopEleToHPReq;
    private final Trigger trg_teleopIntakeReq;
    private final Trigger trg_teleopL1Req; 
    private final Trigger trg_teleopL2Req; 
    private final Trigger trg_teleopL3Req; 
    private final Trigger trg_teleopL4Req; 
    private final Trigger trg_teleopScoreReq;
    private final Trigger trg_dealgaeL2Req;
    private final Trigger trg_dealgaeL3Req;
    private final Trigger trg_climbPrepReq;
    private final Trigger trg_climbBumpButton;
    private final Trigger trg_climbingReq;
    /* simTrigs */
    public final Trigger simTrg_hasCoral = new Trigger(stateEventLoop, () -> m_simHasCoral);

    /* teleopTrgs: overrides */
    private final Trigger trg_inOverride;
    /* sim transitions */
    private final Trigger simTransTrg_intook = new Trigger(stateEventLoop, () -> m_simIntook);
    private final Trigger simTransTrg_scored = new Trigger(stateEventLoop, () -> m_simScored);
    /* Frsies Transition Trigs */
    private final Trigger transTrg_eleNearSetpt; // used for any ele mvmt state
    private final Trigger transTrg_eleNearHP;
    private final Trigger transTrg_eleNearL1;
    private final Trigger transTrg_eleNearL2;
    private final Trigger transTrg_eleNearL3;
    private final Trigger transTrg_eleNearL4;
    private final Trigger transTrg_topSensor;
    private final Trigger transTrg_botSensor;

    /* states */
    public final Trigger stateTrg_idle = new Trigger(stateEventLoop, () -> m_state == State.IDLE);
    public final Trigger stateTrg_eleToHP = new Trigger(stateEventLoop, () -> m_state == State.ELE_TO_HP);
    public final Trigger stateTrg_preIntaking = new Trigger(stateEventLoop, () -> m_state == State.PRE_INTAKE);
    public final Trigger stateTrg_intaking = new Trigger(stateEventLoop, () -> m_state == State.INTAKING);
    public final Trigger stateTrg_slowIntake = new Trigger(stateEventLoop, () -> m_state == State.SLOW_INTAKE);
    public final Trigger stateTrg_intook = new Trigger(stateEventLoop, () -> m_state == State.INTOOK);
    public final Trigger stateTrg_eleToL1 = new Trigger(stateEventLoop, () -> m_state == State.ELE_TO_L1);
    public final Trigger stateTrg_eleToL2 = new Trigger(stateEventLoop, () -> m_state == State.ELE_TO_L2);
    public final Trigger stateTrg_eleToL3 = new Trigger(stateEventLoop, () -> m_state == State.ELE_TO_L3);
    public final Trigger stateTrg_eleToL4 = new Trigger(stateEventLoop, () -> m_state == State.ELE_TO_L4);
    public final Trigger stateTrg_scoreReady = new Trigger(stateEventLoop, () -> m_state == State.SCORE_READY);
    public final Trigger stateTrg_scoring = new Trigger(stateEventLoop, () -> m_state == State.SCORING);
    public final Trigger stateTrg_scored = new Trigger(stateEventLoop, () -> m_state == State.SCORED);

    public final Trigger trg_toScoreHeight = stateTrg_eleToL1.or(stateTrg_eleToL2).or(stateTrg_eleToL3).or(stateTrg_eleToL4);

    public final Trigger stateTrg_algaeRemovalL2 = new Trigger(stateEventLoop, () -> m_state == State.ALGAE_FINGER_L2);
    public final Trigger stateTrg_algaeRemovalL3 = new Trigger(stateEventLoop, () -> m_state == State.ALGAE_FINGER_L3);

    public final Trigger stateTrg_eleToClimb = new Trigger(stateEventLoop, () -> m_state == State.ELE_TO_CLIMB);
    public final Trigger stateTrg_climbReady = new Trigger(stateEventLoop, () -> m_state == State.CLIMB_READY);
    public final Trigger stateTrg_climbing = new Trigger (stateEventLoop, () -> m_state == State.CLIMBING);
    public final Trigger stateTrg_climbed = new Trigger(stateEventLoop, () -> m_state == State.CLIMBED);

    /* sm odds & ends */
    private final DoubleConsumer m_driverRumbler;
    private final Trigger trg_hasCoral;

    /* loggin' */
    private DoubleLogger log_stateIdx = WaltLogger.logDouble(kLogTab, "state idx", PubSubOption.sendAll(true));
    private StringLogger log_stateName = WaltLogger.logString(kLogTab, "state name", PubSubOption.sendAll(true));
    
    /* logs: state trans */
    private BooleanLogger log_autonToHPReq = WaltLogger.logBoolean(kLogTab, "AUTON to HP req");
    private BooleanLogger log_autonScoreReq = WaltLogger.logBoolean(kLogTab, "AUTON score req");

    private BooleanLogger log_teleopToHPReq = WaltLogger.logBoolean(kLogTab, "TELEOP to HP req");
    private BooleanLogger log_teleopIntakeReq = WaltLogger.logBoolean(kLogTab, "TELEOP intake req");
    private BooleanLogger log_teleopScoreReq = WaltLogger.logBoolean(kLogTab, "TELEOP score req");
   
    private BooleanLogger log_eleAtSetpt = WaltLogger.logBoolean(kLogTab, "ele at setpoint");
    private BooleanLogger log_topSensor = WaltLogger.logBoolean(kLogTab, "top beam break");
    private BooleanLogger log_botSensor = WaltLogger.logBoolean(kLogTab, "bot beam break");
    private BooleanLogger log_eleToL1Req = WaltLogger.logBoolean(kLogTab, "ele to lvl 1 req");
    private BooleanLogger log_eleToL2Req = WaltLogger.logBoolean(kLogTab, "ele to lvl 2 req");
    private BooleanLogger log_eleToL3Req = WaltLogger.logBoolean(kLogTab, "ele to lvl 3 req");
    private BooleanLogger log_eleToL4Req = WaltLogger.logBoolean(kLogTab, "ele to lvl 4 req");
    private BooleanLogger log_algaeRemovalButton = WaltLogger.logBoolean(kLogTab, "algae removal button");
    private BooleanLogger log_scoringReq = WaltLogger.logBoolean(kLogTab, "score req");

    private BooleanLogger log_eleToL1Near = WaltLogger.logBoolean(kLogTab, "ele to lvl 1 near");
    private BooleanLogger log_eleToL2Near = WaltLogger.logBoolean(kLogTab, "ele to lvl 2 near");
    private BooleanLogger log_eleToL3Near = WaltLogger.logBoolean(kLogTab, "ele to lvl 3 near");
    private BooleanLogger log_eleToL4Near = WaltLogger.logBoolean(kLogTab, "ele to lvl 4 near");

    private BooleanLogger log_hasCoral = WaltLogger.logBoolean(kLogTab, "has coral");
    private BooleanLogger log_l1Toggle = WaltLogger.logBoolean(kLogTab, "l1 toggle");
    /* sim stuff */
    private BooleanLogger log_simIntook = WaltLogger.logBoolean(kLogTab, "SIM intook");
    private BooleanLogger log_simScored = WaltLogger.logBoolean(kLogTab, "SIM scored");

    /* logs: pieces */
    private IntLogger log_coralScored = WaltLogger.logInt(kLogTab, "coral scored");
    private int coralScored = 0;

    public Superstructure(
        Coral coral,
        Finger finger,
        Elevator ele,
        Optional<Vision> cam1,
        Funnel funnel,
        Trigger eleToHPReq,
        Trigger intakeReq,
        Trigger L1Req,
        Trigger L2Req,
        Trigger L3Req,
        Trigger L4Req,
        Trigger scoreReq,
        Trigger algaeRemovalL2Req,
        Trigger algaeRemovalL3Req,
        Trigger climbPrepReq,
        Trigger climbBump,
        Trigger climbingNowReq,
        Trigger inOverride,
        Trigger simTopBeamBreak,
        Trigger simBotBeamBreak,
        DoubleConsumer driverRumbler
    ) {
        m_coral = coral;
        m_finger = finger;
        m_ele = ele;
        m_cam1 = cam1;
        m_funnel = funnel;
        
        /* state change trigs */
        transTrg_eleNearSetpt = m_ele.remoteAtSetpointTrigger(stateEventLoop);
        transTrg_eleNearHP = m_ele.remoteAtSetpointTrigger(stateEventLoop, EleHeight.HP.rotations);
        transTrg_eleNearL1 = m_ele.remoteAtSetpointTrigger(stateEventLoop, EleHeight.L1.rotations);
        transTrg_eleNearL2 = m_ele.remoteAtSetpointTrigger(stateEventLoop, EleHeight.L2.rotations);
        transTrg_eleNearL3 = m_ele.remoteAtSetpointTrigger(stateEventLoop, EleHeight.L3.rotations);
        transTrg_eleNearL4 = m_ele.remoteAtSetpointTrigger(stateEventLoop, EleHeight.L4.rotations);

        if (!Robot.isSimulation()) {
            transTrg_topSensor = m_coral.topBeamBreakRemoteTrigger(stateEventLoop);
            transTrg_botSensor = m_coral.botBeamBreakRemoteTrigger(stateEventLoop);
        } else {
            transTrg_topSensor = simTopBeamBreak;
            transTrg_botSensor = simBotBeamBreak;
        }

        /* teleop trigs */
        trg_teleopEleToHPReq = eleToHPReq;
        trg_teleopIntakeReq = intakeReq;
        trg_teleopL1Req = L1Req;
        trg_teleopL2Req = L2Req;
        trg_teleopL3Req = L3Req;
        trg_teleopL4Req = L4Req;
        trg_teleopScoreReq = scoreReq;
        trg_dealgaeL2Req = algaeRemovalL2Req;
        trg_dealgaeL3Req = algaeRemovalL3Req;
        trg_climbPrepReq = climbPrepReq;
        trg_climbBumpButton = climbBump;
        trg_climbingReq = climbingNowReq;
        /* overrides */
        trg_hasCoral = transTrg_botSensor.or(transTrg_topSensor).or(simTrg_hasCoral);
        trg_inOverride = inOverride;

        /* binded things */
        m_driverRumbler = driverRumbler;

        configureStateTransitions();
        configureSimTransitions();
        configureStateActions();
    }

    private Command takeCam1Snapshots() {
        return Commands.runOnce(() -> {
            if (m_cam1.isPresent()) {
                m_cam1.get().takeBothSnapshots();
            }
        });
    }
    
    private void configureStateTransitions() {
        (stateTrg_idle.and(trg_teleopEleToHPReq).and(trg_inOverride.negate()).and(RobotModeTriggers.teleop()))
            .onTrue(changeStateCmd(State.ELE_TO_HP));
        (stateTrg_eleToHP.debounce(0.08).and(trg_inOverride.negate()).and(RobotModeTriggers.teleop()))
            .onTrue(changeStateCmd(State.PRE_INTAKE));
        (stateTrg_preIntaking.and(trg_inOverride.negate().and(trg_teleopIntakeReq).and(RobotModeTriggers.teleop())))
            .onTrue(changeStateCmd(State.INTAKING));

        (stateTrg_intaking.and(trg_inOverride.negate()).and(transTrg_topSensor))
            .onTrue(changeStateCmd(State.SLOW_INTAKE));
        (stateTrg_intaking.and(trg_inOverride.negate()).and(transTrg_botSensor))
            .onTrue(changeStateCmd(State.INTOOK));
        (stateTrg_slowIntake.and(trg_inOverride.negate()).and(transTrg_botSensor))
            .onTrue(changeStateCmd(State.INTOOK));
    
        // Teleop Requests
        (trg_hasCoral.and(trg_inOverride.negate()).and(trg_teleopL1Req).and(RobotModeTriggers.teleop()))
            .onTrue(changeStateCmd(State.ELE_TO_L1));
        (trg_hasCoral.and(trg_inOverride.negate()).and(trg_teleopL2Req).and(RobotModeTriggers.teleop()))
            .onTrue(changeStateCmd(State.ELE_TO_L2));
        (trg_hasCoral.and(trg_inOverride.negate()).and(trg_teleopL3Req).and(RobotModeTriggers.teleop()))
            .onTrue(changeStateCmd(State.ELE_TO_L3));
        (trg_hasCoral.and(trg_inOverride.negate()).and(trg_teleopL4Req).and(RobotModeTriggers.teleop()))
            .onTrue(changeStateCmd(State.ELE_TO_L4));
        (trg_toScoreHeight.and(trg_inOverride.negate()).debounce(0.08).and(transTrg_eleNearSetpt).and(RobotModeTriggers.teleop()))
            .onTrue(changeStateCmd(State.SCORE_READY));
    
        (stateTrg_scoreReady.and(trg_inOverride.negate()).and(trg_teleopScoreReq).and(RobotModeTriggers.teleop())) 
            .onTrue(changeStateCmd(State.SCORING));
        (stateTrg_scoring.and(trg_inOverride.negate()).and(transTrg_botSensor.negate())) 
            .onTrue(changeStateCmd(State.SCORED));

        (stateTrg_scored.and(trg_inOverride.negate()).debounce(0.2))
            .onTrue(changeStateCmd(State.ELE_TO_HP));

        // Auton Reqs
        // (stateTrg_idle.and(trg_autonEleToHPReq).and(RobotModeTriggers.autonomous()))
        //     .onTrue(changeStateCmd(State.ELE_TO_HP));
        (stateTrg_eleToHP.and(transTrg_eleNearHP).and(RobotModeTriggers.autonomous()))
            .onTrue(changeStateCmd(State.INTAKING));
        (trg_hasCoral.and(stateTrg_intaking.negate()).and(trg_autonL1Req).and(RobotModeTriggers.autonomous()))
            .onTrue(changeStateCmd(State.ELE_TO_L1));
        (trg_hasCoral.and(stateTrg_intaking.negate()).and(trg_autonL2Req).and(RobotModeTriggers.autonomous()))
            .onTrue(changeStateCmd(State.ELE_TO_L2));
        (trg_hasCoral.and(stateTrg_intaking.negate()).and(trg_autonL3Req).and(RobotModeTriggers.autonomous()))
            .onTrue(changeStateCmd(State.ELE_TO_L3));
        (trg_hasCoral.and(stateTrg_intaking.negate()).and(trg_autonL4Req).and(RobotModeTriggers.autonomous()))
            .onTrue(changeStateCmd(State.ELE_TO_L4));
        (stateTrg_eleToL1.and(transTrg_eleNearL1).and(RobotModeTriggers.autonomous()))
            .onTrue(changeStateCmd(State.SCORE_READY));
        (stateTrg_eleToL2.and(transTrg_eleNearL2).and(RobotModeTriggers.autonomous()))
            .onTrue(changeStateCmd(State.SCORE_READY));
        (stateTrg_eleToL3.and(transTrg_eleNearL3).and(RobotModeTriggers.autonomous()))
            .onTrue(changeStateCmd(State.SCORE_READY));
        (stateTrg_eleToL4.and(transTrg_eleNearL4).and(RobotModeTriggers.autonomous()))
            .onTrue(changeStateCmd(State.SCORE_READY));
        (stateTrg_scoreReady.and(trg_autonScoreReq).and(RobotModeTriggers.autonomous())) 
            .onTrue(changeStateCmd(State.SCORING));

        (trg_hasCoral.negate().and(trg_dealgaeL2Req)).and(RobotModeTriggers.teleop())
            .onTrue(changeStateCmd(State.ALGAE_FINGER_L2));
        (trg_hasCoral.negate().and(trg_dealgaeL3Req)).and(RobotModeTriggers.teleop())
            .onTrue(changeStateCmd(State.ALGAE_FINGER_L3));
        (stateTrg_algaeRemovalL2.and(trg_dealgaeL2Req.negate()).and(RobotModeTriggers.teleop()))
            .onTrue(changeStateCmd(State.ELE_TO_HP));
        (stateTrg_algaeRemovalL3.and(trg_dealgaeL3Req.negate()).and(RobotModeTriggers.teleop()))
            .onTrue(changeStateCmd(State.ELE_TO_HP));

        (m_funnel.trg_atCurrLim).or(transTrg_topSensor)
            .onTrue(driverRumble(kRumbleIntensity, kRumbleTimeoutSecs));
    }

    // cuz i dont have a joystick myself and ill usually use sim at home, im going to automate everything
    // stuff will prolly get added as i need them
    private void configureSimTransitions() {
        // (stateTrg_idle.and(() -> Utils.isSimulation()).and(RobotModeTriggers.teleop())).debounce(1) 
        //     .onTrue(
        //         Commands.runOnce(() -> m_eleToHPStateTransReq = true)
        //     );
        (stateTrg_intaking.and(() -> Utils.isSimulation())).debounce(0.5)
            .onTrue(simIntook());
        // (stateTrg_intook.and(() -> Utils.isSimulation())).debounce(1)
        //     .onTrue(
        //         Commands.sequence(
        //             Commands.runOnce(() -> m_eleToL4Req = true)
        //         )
        //     );
        (stateTrg_scoreReady.and(() -> Utils.isSimulation())).debounce(0.5)
            .onTrue(simScored());
        // (stateTrg_scoring.and(() -> Utils.isSimulation()).and(RobotModeTriggers.teleop())).debounce(0.5)
        //     .onTrue(simScored());

        simTransTrg_intook
            .onTrue(
                Commands.sequence(
                    changeStateCmd(State.INTOOK),
                    Commands.runOnce(() -> m_simIntook = false)
                )
            );

        simTransTrg_scored
            .onTrue(
                Commands.sequence(
                    changeStateCmd(State.SCORING),
                    Commands.waitSeconds(.5),
                    changeStateCmd(State.SCORED),
                    Commands.runOnce(() -> m_simScored = false)
                ));
    }

    private void configureStateActions() {
        stateTrg_idle
            .onTrue(resetEverything());

        stateTrg_eleToHP
            .onTrue(
                Commands.parallel(
                    m_ele.toHeightCoral(() -> HP),
                    Commands.runOnce(() -> m_autonEleToHPReq = false)
                )
            );

        stateTrg_intaking
            .onTrue(
                Commands.sequence(
                    Commands.parallel(
                        m_funnel.fast(),
                        m_coral.fastIntake()
                    ),
                    Commands.waitUntil(m_coral.trg_topBeamBreak)
                )
            );

        stateTrg_slowIntake
            .onTrue(
                Commands.sequence(
                    Commands.deadline(Commands.waitUntil(m_coral.trg_botBeamBreak),
                        Commands.repeatingSequence(
                            Commands.parallel(
                                m_coral.slowIntake()
                            ),
                            Commands.waitSeconds(1),
                            Commands.parallel(
                                m_coral.slowIntakeReversal()
                            ),
                            Commands.waitSeconds(0.05)
                        )
                    )
                ).alongWith(takeCam1Snapshots())
            );
        
        stateTrg_intook
            .onTrue(
                Commands.parallel(
                    m_funnel.stopCmd(),
                    m_coral.stopCmd()
                ));
        
        stateTrg_eleToL1
            .onTrue(
                Commands.parallel(
                    m_ele.toHeightCoral(() -> L1),
                    Commands.runOnce(() -> m_autonEleToL1Req = false),
                    Commands.runOnce(() -> m_l1Toggle = true)
                )
            );

        stateTrg_eleToL2
            .onTrue(
                Commands.parallel(
                    m_ele.toHeightCoral(() -> L2), 
                    Commands.runOnce(() -> m_autonEleToL2Req = false)
                )
            );

        stateTrg_eleToL3
            .onTrue(
                Commands.parallel(
                    m_ele.toHeightCoral(() -> L3),
                    Commands.runOnce(() -> m_autonEleToL3Req = false)
                )
            );

        stateTrg_eleToL4
            .onTrue(
                Commands.parallel(
                    m_ele.toHeightCoral(() -> L4),
                    Commands.runOnce(() -> m_autonEleToL4Req = false)
                )
            );

        stateTrg_scoring.and(trg_l1Toggle.negate())
            .onTrue(
                Commands.sequence(
                    m_coral.score(),
                    Commands.waitUntil(m_coral.trg_botBeamBreak.negate()),
                    m_coral.stopCmd()
                ).alongWith(takeCam1Snapshots())
            );
        
        stateTrg_scoring.and(trg_l1Toggle)
            .onTrue(
                Commands.sequence(
                    m_coral.slowScore(),
                    Commands.waitSeconds(0.164),
                    m_ele.toHeightCoral(() -> L2),
                    m_finger.l1HelperCmd(),
                    Commands.waitSeconds(0.18),
                    m_finger.inCmd()
                )
            );

        stateTrg_scored
            .onTrue(
                resetTriggers()
            );

        stateTrg_algaeRemovalL2
            .onTrue(
                Commands.parallel(
                    m_ele.toHeightAlgae(() -> AlgaeHeight.L2),
                    algaeRemoval()
                )
            );

        stateTrg_algaeRemovalL3
            .onTrue(
                Commands.parallel(
                    m_ele.toHeightAlgae(() -> AlgaeHeight.L3),
                    algaeRemoval()
                )
            );
    }

    /* state change methods */
    private Command changeStateCmd(State newState) {
        return Commands.runOnce(() -> {
            if (newState == m_state) {
                return;
            }
            System.out.println("[SUPER] Changing state from (" + m_state.name + ") to (" + newState.name + ")");
            m_state = newState;
            log_stateIdx.accept(m_state.idx);
            log_stateName.accept(m_state.name);
            if (newState == State.SCORED) {
                coralScored++;
                log_coralScored.accept(coralScored);
            }
        });
    }

    public Command forceIdle() {
        if(
            m_state == State.ELE_TO_CLIMB ||
            m_state == State.CLIMB_READY ||
            m_state == State.CLIMBING
        ) {
            return Commands.none();
        } else {
            return (changeStateCmd(State.IDLE));
        }
    }

    public Command forcetoHP() {
        return (changeStateCmd(State.ELE_TO_HP));
    }
    public Command forceStateToIntake() {
        return (changeStateCmd(State.INTAKING));
    }
   public Command forceShoot() {
        return m_coral.score();
    }
    public Command changeStateToScored() {
        return (changeStateCmd(State.SCORED));
    }
    public Command forceL1() {
        return (changeStateCmd(State.ELE_TO_L1));
    }
    public Command forceL2() {
        return (changeStateCmd(State.ELE_TO_L2));
    }
    public Command forceL3() {
        return (changeStateCmd(State.ELE_TO_L3));
    }
    public Command forceL4() {
        return (changeStateCmd(State.ELE_TO_L4));
    }

    /* methods that Actually Do Things */
    public Command resetEverything() {
        return Commands.sequence(
            m_coral.stopCmd(),
            m_funnel.stopCmd(),
            Commands.print("in reset everything"),
            m_ele.toHeightCoral(() -> HOME),
            driverRumble(0, kRumbleTimeoutSecs)
        );
    }

    public Command algaeRemoval() {
        return baseAlgaeRemoval();
    }

    public Command baseAlgaeRemoval() {
        return Commands.parallel(
            m_finger.algaeDescoreCmd(),
            m_coral.runWheelsAlgaeRemoval()
        );
    }

    public Trigger getBottomBeamBreak() {
        return m_coral.trg_botBeamBreak;
    }

    public Trigger getTopBeamBreak() {
        return m_coral.trg_topBeamBreak;
    }

    /* to be used in auton */
    public Command autonEleToHPReq() {
        return Commands.runOnce(() -> m_autonEleToHPReq = true);
    }

    public Command autonEleToL1Req() {
        return Commands.runOnce(() -> m_autonEleToL1Req = true);
    }

    public Command autonEleToL2Req() {
        return Commands.runOnce(() -> m_autonEleToL2Req = true);
    }

    public Command autonEleToL3Req() {
        return Commands.runOnce(() -> m_autonEleToL3Req = true);
    }

    public Command autonEleToL4Req() {
        return Commands.runOnce(() -> m_autonEleToL4Req = true);
    }

    // use this in autonfactory
    public Command autonEleToScoringPosReq(EleHeight height) {
        if(height == L1) {
            return autonEleToL1Req();
        } else if(height == L2) {
            return autonEleToL2Req();
        } else if(height == L3) {
            return autonEleToL3Req();
        } else if(height == L4) {
            return autonEleToL4Req();
        } else {
            return Commands.print("invalid height for auton score req. wanted " + height);
        }
    }

    public Command resetTriggers() {
        return Commands.runOnce(() -> {
             m_autonScoreReq = false;
             m_autonEleToL1Req = false;
             m_autonEleToL2Req = false;
             m_autonEleToL3Req = false;
             m_autonEleToL4Req = false;
             m_l1Toggle = false;
        });
    }

    public Command autonPreloadReq() {
        return (changeStateCmd(State.INTOOK));
    }

    public Command autonScoreReq() {
        return Commands.sequence(
            Commands.print("AutonScoreReq!!!"),
            Commands.runOnce(() -> m_autonScoreReq = true)
        );
    }

    /* to be used in sim */
    public Command simIntook() {
        return Commands.runOnce(() -> m_simIntook = true);
    }

    public Command simScored() {
        return Commands.runOnce(() -> m_simScored = true);
    }

    public Command simHasCoralToggle() {
        if(Robot.isSimulation()) {
            return Commands.runOnce(() -> m_simHasCoral = !m_simHasCoral);
        } else {
            return Commands.none();
        }
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
        log_autonToHPReq.accept(trg_autonEleToHPReq);
        log_autonScoreReq.accept(trg_autonScoreReq);

        log_teleopToHPReq.accept(trg_teleopEleToHPReq);
        log_teleopIntakeReq.accept(trg_teleopIntakeReq);
        log_teleopScoreReq.accept(trg_teleopScoreReq);

        log_eleToL1Req.accept(trg_teleopL1Req.or(trg_autonL1Req));
        log_eleToL2Req.accept(trg_teleopL2Req.or(trg_autonL2Req));
        log_eleToL3Req.accept(trg_teleopL3Req.or(trg_autonL3Req));
        log_eleToL4Req.accept(trg_teleopL4Req.or(trg_autonL4Req));

        log_eleToL1Near.accept(transTrg_eleNearL1);
        log_eleToL2Near.accept(transTrg_eleNearL2);
        log_eleToL3Near.accept(transTrg_eleNearL3);
        log_eleToL4Near.accept(transTrg_eleNearL4);

        log_algaeRemovalButton.accept(trg_dealgaeL2Req.or(trg_dealgaeL3Req));

        log_l1Toggle.accept(trg_l1Toggle);
    }

    public void logStateChangeReqs() {
        // TODO: readd logging back in

        log_eleAtSetpt.accept(transTrg_eleNearSetpt);
        log_topSensor.accept(transTrg_topSensor);
        log_botSensor.accept(transTrg_botSensor);

        log_hasCoral.accept(trg_hasCoral);
        log_scoringReq.accept(trg_teleopScoreReq.or(trg_autonScoreReq));
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

        if(Robot.isSimulation()) {
            logSimThings();
        }
    }

    public static enum State {
        IDLE(0, "idle"),
        ELE_TO_HP(1, "ele to intake"),
        PRE_INTAKE(2, "pre intake"),
        INTAKING(3, "intaking"),
        SLOW_INTAKE(4, "slow intake"),
        INTOOK(5, "intook"),
        ELE_TO_L1(6.1, "ele to L1"),
        ELE_TO_L2(6.2, "ele to L2"),
        ELE_TO_L3(6.3, "ele to L3"),
        ELE_TO_L4(6.4, "ele to L4"),
        SCORE_READY(7, "score ready"),
        SCORING(8, "scoring"),
        SCORED(9, "scored"),
        
        ALGAE_FINGER_L2(10.2, "algae finger"),
        ALGAE_FINGER_L3(10.3, "algae finger"),

        ELE_TO_CLIMB(11, "ele to climb"),
        CLIMB_READY(12, "climb ready"),
        CLIMBING(13.1, "climbing finger"),
        CLIMBED(14, "climbed");

        public final double idx;
        public final String name;
  
        private State(double index, String _name) {
            idx = index;
            name = _name;
        }
    }
}
