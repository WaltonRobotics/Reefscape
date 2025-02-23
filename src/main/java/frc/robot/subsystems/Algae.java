package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
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
import frc.util.WaltLogger.DoubleLogger;

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
    private boolean m_intakeIsCoast = true;
    private GenericEntry nte_intakeIsCoast;

    private final DoubleConsumer m_manipRumbler;

    private double m_desiredWristRotations = 0;
    private final DoubleLogger log_desiredAngleDegs = WaltLogger.logDouble(kLogTab, "desiredAngleDegs");

    private MotionMagicExpoVoltage m_MMEVRequest = new MotionMagicExpoVoltage(0).withEnableFOC(true);

    private State m_state;
    public final EventLoop stateEventLoop = new EventLoop();

    private final Trigger trg_groundReq;
    private final Trigger trg_nearSetPt = new Trigger(() -> nearSetpoint());
    private final Trigger trg_intakeReq;
    private final Trigger trg_hasAlgae = new Trigger(() -> isAlgaeThere());
    private final Trigger trg_processorReq;
    private final Trigger trg_shootReq;

    public final Trigger stateTrg_idle = new Trigger(stateEventLoop, () -> m_state == State.IDLE);
    public final Trigger stateTrg_toGround = new Trigger(stateEventLoop, () -> m_state == State.TO_GROUND);
    public final Trigger stateTrg_ground = new Trigger(stateEventLoop, () -> m_state == State.GROUND);
    public final Trigger stateTrg_intaking = new Trigger(stateEventLoop, () -> m_state == State.INTAKING);
    public final Trigger stateTrg_intook = new Trigger(stateEventLoop, () -> m_state == State.INTOOK);
    public final Trigger stateTrg_home = new Trigger(stateEventLoop, () -> m_state == State.HOME);
    public final Trigger stateTrg_toProcessor = new Trigger(stateEventLoop, () -> m_state == State.TO_PROCESSOR);
    public final Trigger stateTrg_processor = new Trigger(stateEventLoop, () -> m_state == State.PROCESSOR);
    public final Trigger stateTrg_shooting = new Trigger(stateEventLoop, () -> m_state == State.SHOOTING);
    public final Trigger stateTrg_shot = new Trigger(stateEventLoop, () -> m_state == State.SHOT);

    private boolean m_isHomed = false;
    private Debouncer m_currentDebouncer = new Debouncer(0.25, DebounceType.kRising);
    private Debouncer m_velocityDebouncer = new Debouncer(0.125, DebounceType.kRising);
    private BooleanSupplier m_currentSpike = () -> m_wrist.getStatorCurrent().getValueAsDouble() > 5.0; 
    private VoltageOut zeroingVoltageCtrlReq = new VoltageOut(-0.75);
    private BooleanSupplier m_veloIsNearZero = () -> Math.abs(m_wrist.getVelocity().getValueAsDouble()) < 0.01;

    public Algae(
        Trigger groundReq, 
        Trigger intakeReq, 
        Trigger processorReq, 
        Trigger shootReq, 
        DoubleConsumer manipRumbler,
        DoubleSupplier overrideAngle
    ) {
        m_wrist.getConfigurator().apply(kWristConfiguration);
        m_intake.getConfigurator().apply(kIntakeConfiguration);

        nte_wristIsCoast = Shuffleboard.getTab(kLogTab)
                  .add("wrist coast", false)
                  .withWidget(BuiltInWidgets.kToggleSwitch)
                  .getEntry();
        nte_intakeIsCoast = Shuffleboard.getTab(kLogTab)
                  .add("intake coast", false)
                  .withWidget(BuiltInWidgets.kToggleSwitch)
                  .getEntry();
        
        m_state = State.IDLE;

        trg_groundReq = groundReq;
        trg_intakeReq = intakeReq;
        trg_processorReq = processorReq;
        trg_shootReq = shootReq;

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
            toAngle(WristPos.HOME),
            Commands.runOnce(() -> setWheelAction(0))
        );
    }

    private void configureStateTransitions() {
        (stateTrg_idle.and(trg_groundReq))
            .onTrue(Commands.runOnce(() -> m_state = State.TO_GROUND));
        (stateTrg_toGround.and(trg_nearSetPt))
            .onTrue(Commands.runOnce(() -> m_state = State.GROUND));
        (stateTrg_ground.and(trg_intakeReq))
            .onTrue(Commands.runOnce(() -> m_state = State.INTAKING));
        (stateTrg_intaking.and(trg_hasAlgae))
            .onTrue(Commands.runOnce(() -> m_state = State.INTOOK));
        (stateTrg_intook.and(trg_nearSetPt))
            .onTrue(Commands.runOnce(() -> m_state = State.HOME));
        (stateTrg_home.and(trg_processorReq))
            .onTrue(Commands.runOnce(() -> m_state = State.TO_PROCESSOR));
        (stateTrg_toProcessor.and(trg_nearSetPt))
            .onTrue(Commands.runOnce(() -> m_state = State.PROCESSOR));
        (stateTrg_processor.and(trg_shootReq))
            .onTrue(Commands.runOnce(() -> m_state = State.SHOOTING));
        (stateTrg_shooting.and(trg_hasAlgae.negate()))
            .onTrue(Commands.runOnce(() -> m_state = State.SHOT));
        (stateTrg_shot)
            .onTrue(Commands.runOnce(() -> m_state = State.IDLE));
    }

    private void configureStateActions() {
        (stateTrg_idle)
            .onTrue(Commands.runOnce(() -> resetEverything()));
        (stateTrg_toGround)
            .onTrue(toAngle(WristPos.GROUND));
        (stateTrg_ground)
            .onTrue(
                Commands.parallel(
                    /* TODO: write a wristSpring method. for now, that doesn't exist. */
                    manipRumble(kRumbleIntensity, kRumbleTimeoutSecs)
                )
            );
        (stateTrg_intaking)
            .onTrue(intake());
        // intook has no actions so far
        (stateTrg_home)
            .onTrue(toAngle(WristPos.HOME)); // TODO: make this automatic w/ wristSpring. then, i just need to unset wristSpringMode.
        (stateTrg_toProcessor)
            .onTrue(toAngle(WristPos.PROCESSOR));
        (stateTrg_processor)
            .onTrue(manipRumble(kRumbleIntensity, kRumbleTimeoutSecs));
        (stateTrg_shooting)
            .onTrue(shoot());
        (stateTrg_shot)
            .onTrue(toAngle(WristPos.HOME)); 
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
        m_wrist.setNeutralMode(coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
    }

    public Command toAngle(WristPos wristPos) {
        return toAngle(wristPos.angleDegs);
    }

    public Command toAngle(double angleDegs) {
        return runOnce(
            () -> {
                m_desiredWristRotations = angleDegs;
                m_MMEVRequest = m_MMEVRequest.withPosition(m_desiredWristRotations);
                m_wrist.setControl(m_MMEVRequest);}
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
        double diff = m_MMEVRequest.Position - m_wrist.getPosition().getValueAsDouble();
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
            System.out.println("Zeroed Elevator!!!");
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
        return Commands.startEnd(
            () -> setWheelAction(12),
            () -> setWheelAction(0)
        );
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
        return m_intake.getStatorCurrent().getValueAsDouble() >= kHasAlgaeCurrent;
    }

    @Override
    public void periodic() {
        stateEventLoop.poll();

        m_wristIsCoast = nte_wristIsCoast.getBoolean(false);
        m_intakeIsCoast = nte_intakeIsCoast.getBoolean(true);

        // setWristCoast(m_wristIsCoast);
        // setIntakeCoast(m_intakeIsCoast);

        log_desiredAngleDegs.accept(m_desiredWristRotations);
    }

    public enum State {
        IDLE(0),
        TO_GROUND(1),
        GROUND(2),
        INTAKING(3),
        INTOOK(4),
        HOME(5),
        TO_PROCESSOR(6),
        PROCESSOR(7),
        SHOOTING(8),
        SHOT(9),
        OVERRIDE(10);

        public int idx;
        private State(int index) {
            idx = index;
        }
    }

    public enum WristPos {
        HOME(0),
        GROUND(0.28),
        PROCESSOR(0.14);

        public double angleDegs;
        private WristPos(double angle) {
            angleDegs = angle;
        }
    }
}