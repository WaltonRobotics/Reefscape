package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;
import frc.util.WaltLogger.IntLogger;
import frc.util.WaltLogger.StringLogger;

import static frc.robot.Constants.kRumbleIntensity;
import static frc.robot.Constants.kRumbleTimeoutSecs;
import static frc.robot.Constants.AlgaeK.*;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
 
public class Algae extends SubsystemBase {
    private final TalonFX m_wrist = new TalonFX(kWristCANID, TunerConstants.kCANBus); // I KNOW this is not a wrist its a shoulder, but im going to call it a wrist.
    private final TalonFX m_intake = new TalonFX(kIntakeCANID, TunerConstants.kCANBus);

    private boolean m_wristIsCoast = false; 
    private GenericEntry nte_wristIsCoast;

    private final DoubleConsumer m_manipRumbler;

    private double m_desiredWristRotations = 0;
    private final DoubleLogger log_desiredAngleDegs = WaltLogger.logDouble(kLogTab, "desiredAngleDegs");

    private DynamicMotionMagicVoltage m_MMVRequest = new DynamicMotionMagicVoltage(
        0, kWristMMVelo, kWristMMAccel, kWristMMJerk
    ).withEnableFOC(true);

    private State m_state;
    public final EventLoop stateEventLoop = new EventLoop();

    private final Debouncer hasAlgaeDebouncer = new Debouncer(0.5, DebounceType.kRising);

    private final Trigger trg_groundReq;
    private final Trigger trg_hasAlgae = new Trigger(() -> isAlgaeThere());
    private final Trigger trg_processorReq;

    public final Trigger stateTrg_idle = new Trigger(stateEventLoop, () -> m_state == State.IDLE);
    public final Trigger stateTrg_intaking = new Trigger(stateEventLoop, () -> m_state == State.INTAKING);
    public final Trigger stateTrg_home = new Trigger(stateEventLoop, () -> m_state == State.HOME);

    private boolean m_isHomed = false;
    private Debouncer m_currentDebouncer = new Debouncer(0.25, DebounceType.kRising);
    private Debouncer m_velocityDebouncer = new Debouncer(0.125, DebounceType.kRising);
    private BooleanSupplier m_currentSpike = () -> m_wrist.getStatorCurrent().getValueAsDouble() > 5.0; 
    private VoltageOut zeroingVoltageCtrlReq = new VoltageOut(-0.75);
    private BooleanSupplier m_veloIsNearZero = () -> Math.abs(m_wrist.getVelocity().getValueAsDouble()) < 0.01;

    private IntLogger log_stateIdx = WaltLogger.logInt(kLogTab, "Algae State idx");
    private StringLogger log_stateName = WaltLogger.logString(kLogTab, "Algae State name");
    private BooleanLogger log_hasAlgae = WaltLogger.logBoolean(kLogTab, "trgHasAlgae");
    private DoubleLogger log_actualAngle = WaltLogger.logDouble(kLogTab, "Algae angle");

    public Algae(
        Trigger groundReq, 
        Trigger processorReq, 
        Trigger shootReq, 
        DoubleConsumer manipRumbler
    ) {
        m_wrist.getConfigurator().apply(kWristConfiguration);
        m_intake.getConfigurator().apply(kIntakeConfiguration);

        nte_wristIsCoast = Shuffleboard.getTab(kLogTab)
                  .add("wrist coast", false)
                  .withWidget(BuiltInWidgets.kToggleSwitch)
                  .getEntry();

        m_state = State.IDLE;

        trg_groundReq = groundReq;
        trg_processorReq = processorReq;

        m_manipRumbler = manipRumbler;

        setDefaultCommand(currentSenseHoming());

        configureStateTransitions();
        configureStateActions();
    }

    /*
     * used when u go to idle mode
     */
    // i deffo feel like this isnt all that i need to reset but i cant think of anything else rn
    private Command resetEverything() {
        return Commands.parallel(
            toAngle(WristPos.IDLE),
            Commands.runOnce(() -> setWheelAction(0))
        );
    }

    private void configureStateTransitions() {
        (stateTrg_idle.and(trg_groundReq))
            .onTrue(changeStateCmd(State.INTAKING));
        (stateTrg_intaking.and(trg_hasAlgae))
            .onTrue(changeStateCmd(State.HOME));
        (stateTrg_home.and(trg_processorReq))
            .onTrue(changeStateCmd(State.IDLE));
    }

    private void configureStateActions() {
        (stateTrg_idle)
            .onTrue(resetEverything());
        (stateTrg_intaking)
            .onTrue(
                Commands.parallel(
                    toAngle(WristPos.GROUND),
                    intake()
                )
            );
        (stateTrg_home)
            .onTrue(toAngle(WristPos.STORE, true)); // TODO: make this automatic w/ wristSpring. then, i just need to unset wristSpringMode.
    }

    private Command manipRumble(double intensity, double secs) {
        return Commands.run(
            () -> m_manipRumbler.accept(intensity)
        ).withTimeout(secs);
    }

    public int getStateIdx() {
        return m_state.idx;
    }

    // WRIST SCHTUFFS
    public void setWristCoast(boolean coast) {
        if (m_wristIsCoast != coast) {
            m_wristIsCoast = coast;
            m_wrist.setNeutralMode(m_wristIsCoast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
        }
    }

    public Command toAngle(WristPos wristPos) {
        return toAngle(wristPos, false);
    }

    public Command toAngle(WristPos wristPos, boolean slowPls) {
        return toAngle(wristPos.angleDegs, slowPls);
    }

    public Command toAngle(double angleDegs, boolean slowPls) {
        return runOnce(
            () -> {
                m_desiredWristRotations = angleDegs;
                m_MMVRequest = m_MMVRequest.withPosition(m_desiredWristRotations);
                var localRequest = m_MMVRequest;
                if (slowPls) {
                    localRequest = localRequest
                        .withVelocity(kWristMMVeloSlow)
                        .withAcceleration(kWristMMAccelSlow);
                }
                m_wrist.setControl(localRequest);}
        ).until(() -> nearSetpoint());
    }

    // testin' only
    public Command testVoltageControl(DoubleSupplier stick) {
        return runEnd(() -> {
            m_wrist.setControl(zeroingVoltageCtrlReq.withOutput(-(stick.getAsDouble()) * 6));
        }, () -> {
            m_wrist.setControl(zeroingVoltageCtrlReq.withOutput(0));
        }
        );
    }

    public boolean nearSetpoint() {
        return nearSetpoint(kAngleTolerance);
    }

    public boolean nearSetpoint(double tolerancePulleyRotations) {
        double diff = m_MMVRequest.Position - m_wrist.getPosition().getValueAsDouble();
        return Math.abs(diff) <= tolerancePulleyRotations;
    }

    public Command currentSenseHoming() {
        Runnable init = () -> {
            m_wrist.setControl(zeroingVoltageCtrlReq.withOutput(-1));
        };
        Runnable execute = () -> {};
        Consumer<Boolean> onEnd = (Boolean interrupted) -> {
            m_wrist.setPosition(0);
            m_wrist.setControl(zeroingVoltageCtrlReq.withOutput(0));
            removeDefaultCommand();
            m_isHomed = true;
            System.out.println("Zeroed Algae!!!");
        };

        BooleanSupplier isFinished = () ->
            m_currentDebouncer.calculate(m_currentSpike.getAsBoolean()) && 
            m_velocityDebouncer.calculate(m_veloIsNearZero.getAsBoolean());

        return new FunctionalCommand(init, execute, onEnd, isFinished, this);
    }

    public boolean getIsHomed() {
        return m_isHomed;
    }

    // INTAKE SCHTUFFS
    public void setIntakeCoast(boolean coast) {
        m_intake.setNeutralMode(coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
    }

    public Command intake() {
        return Commands.runOnce(() -> setWheelAction(12));
    }

    public Command shoot() {
        return Commands.runEnd(
            () -> setWheelAction(-12),
            () -> setWheelAction(0)
        ).until(trg_hasAlgae.negate());
    }

    public void setWheelAction(double destinationVoltage) {
        // should this be a runend? i thought no cuz i didn't want the motor to stop until i told it to
        // ive decided no (watch me be wrong (idt im wrong tho (maybe)))
        m_intake.setVoltage(destinationVoltage);
    }

    public boolean isAlgaeThere() {
        return hasAlgaeDebouncer.calculate(m_intake.getStatorCurrent().getValueAsDouble() >= kHasAlgaeCurrent);
    }

    public Command changeStateCmd(State newState) {
        return Commands.runOnce(() -> {
            if (newState == m_state) {
                return;
            }
            System.out.println("[ALGAE] Changing state from (" + m_state.name + ") to (" + newState.name + ")");
            m_state = newState;
            log_stateIdx.accept(m_state.idx);
            log_stateName.accept(m_state.name);
        });
    }

    public Command toIdleCmd() {
        return changeStateCmd(State.IDLE);
    }

    @Override
    public void periodic() {
        stateEventLoop.poll();
    
        setWristCoast(nte_wristIsCoast.getBoolean(false));

        log_stateIdx.accept(m_state.idx);
        log_stateName.accept(m_state.name);

        log_desiredAngleDegs.accept(m_desiredWristRotations);
        log_hasAlgae.accept(trg_hasAlgae);

        log_actualAngle.accept(m_wrist.getPosition().getValueAsDouble());
        log_desiredAngleDegs.accept(m_desiredWristRotations);
    }

    public enum State {
        IDLE(0, "Idle"),
        INTAKING(1, "Intaking"),
        HOME(2, "Home");

        public int idx;
        public String name;
        private State(int index, String name) {
            idx = index;
            this.name = name;
        }
    }

    public enum WristPos {
        IDLE(0),
        STORE(0.08),
        GROUND(0.28);
        // PROCESSOR(0.14);

        public double angleDegs;
        private WristPos(double angle) {
            angleDegs = angle;
        }
    }
}