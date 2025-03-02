package frc.robot.subsystems;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.Coralk.*;
import static frc.robot.Constants.Coralk.FingerK.*;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;

public class Coral extends SubsystemBase {
    private final TalonFX m_coralMotor = new TalonFX(kCoralMotorCANID);

    private final TalonFXS m_fingerMotor = new TalonFXS(kFingerMotorCANID);
    private PositionVoltage m_PosVoltReq = new PositionVoltage(0);
    private VoltageOut m_voltOutReq = new VoltageOut(0);
    private NeutralOut m_neutralOut = new NeutralOut();
    private VoltageOut zeroingVoltageCtrlReq = new VoltageOut(0);

    private BooleanSupplier m_currentSpike = () -> m_fingerMotor.getStatorCurrent().getValueAsDouble() > 5.0; 
    private BooleanSupplier m_veloIsNearZero = () -> Math.abs(m_fingerMotor.getVelocity().getValueAsDouble()) < 0.01;

    private Debouncer m_currentDebouncer = new Debouncer(0.25, DebounceType.kRising);
    private Debouncer m_velocityDebouncer = new Debouncer(0.125, DebounceType.kRising);

    private final double m_slowIntakeSpeed = 12 * .25;
    private final double m_scoreSpeed = 5;

    private boolean m_isHomed = false;

    // true when beam break brokey
    public DigitalInput m_topBeamBreak = new DigitalInput(kTopBeamBreakChannel);
    public DigitalInput m_botBeamBreak = new DigitalInput(kBotBeamBreakChannel);

    public final Trigger trg_topBeamBreak = new Trigger(() -> !m_topBeamBreak.get());
    public final Trigger trg_botBeamBreak = new Trigger(() -> !m_botBeamBreak.get());

    private final BooleanLogger log_topBeamBreak = WaltLogger.logBoolean(kLogTab, "topBeamBreak");
    private final BooleanLogger log_botBeamBreak = WaltLogger.logBoolean(kLogTab, "botBeamBreak");

    public Coral() {
        m_coralMotor.getConfigurator().apply(kCoralMotorTalonFXConfiguration);
        // m_fingerMotor.getConfigurator().apply(kFingerMotorTalonFXSConfig);

        // setDefaultCommand(currentSenseHoming());
    }

    // good method
    public Command automaticCoralIntake() {
        return Commands.sequence(
            fastIntake().until(trg_topBeamBreak),
            slowIntake().until(trg_botBeamBreak),
            stopCoralMotorCmd()
        );
    }

    public void setCoralCoast(boolean coast) {
        m_coralMotor.setNeutralMode(coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
    }

    private void setCoralMotorAction(double voltage) {
        m_coralMotor.setControl(m_voltOutReq.withOutput(voltage));
    }

    public Command setCoralMotorActionCmd(double voltage) {
        return runOnce(() -> {
            setCoralMotorAction(voltage);
            System.out.println("CoralMotorSet: " + voltage);
        });
    }

    private void stopCoralMotor() {
        m_coralMotor.setControl(m_neutralOut);
    }

    public Command stopCoralMotorCmd() {
        return runOnce(this::stopCoralMotor);
    }

    public Command fastIntake() {
        return setCoralMotorActionCmd(12);
    }

    /*
     * This happens right after the Top Beam Break occurs so that we dont *woosh* the coral out
     */
    public Command slowIntake(){
        return setCoralMotorActionCmd(m_slowIntakeSpeed);
    }

    public Command slowIntakeReversal(){
        return setCoralMotorActionCmd(-m_slowIntakeSpeed);
    }

    public Command score() {
        return setCoralMotorActionCmd(m_scoreSpeed);
    }

    // finger methods
    private void fingerOut() {
        m_fingerMotor.setControl(m_PosVoltReq.withPosition(kParallelToGroundRotations));
    }

    public Command fingerOutCmd() {
        return runOnce(this::fingerOut);
    }

    private void fingerIn() {
        m_fingerMotor.setControl(m_PosVoltReq.withPosition(kMaxAngleRotations));
    }

    public Command fingerInCmd() {
        return runOnce(this::fingerIn);
    }

    public Command testFingerVoltageControl(DoubleSupplier stick) {
        return runEnd(() -> {
            m_fingerMotor.setControl(m_voltOutReq.withOutput(-(stick.getAsDouble()) * 6));
        }, () -> {
            m_fingerMotor.setControl(m_voltOutReq.withOutput(0));
        }
        );
    }

    private void runWheelsAlgaeRemoval() {
        setCoralMotorAction(-m_scoreSpeed);
    }

    public Command runWheelsAlgaeRemovalCmd() {
        return runOnce(this::runWheelsAlgaeRemoval);
    }

    public void setFingerCoast(boolean coast) {
        m_fingerMotor.setNeutralMode(coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
    }

    public Command algaeIntake() {
        return startEnd(
            () -> {
                fingerOut();
                runWheelsAlgaeRemoval();
            }, () -> {
                fingerIn();
                stopCoralMotor();
            }
        );
    }

    public Command currentSenseHoming() {
        Runnable init = () -> {
            m_fingerMotor.getConfigurator().apply(kFingerSoftwareLimitSwitchWithSoftLimitDisableConfig);
            m_fingerMotor.setControl(zeroingVoltageCtrlReq.withOutput(-1));
        };
        Runnable execute = () -> {};
        Consumer<Boolean> onEnd = (Boolean interrupted) -> {
            m_fingerMotor.setPosition(0);
            m_fingerMotor.setControl(zeroingVoltageCtrlReq.withOutput(0));
            removeDefaultCommand();
            m_isHomed = true;
            m_fingerMotor.getConfigurator().apply(kFingerSoftwareLimitSwitchWithSoftLimitEnabledConfig);
            System.out.println("Zeroed Finger!!!");
        };

        BooleanSupplier isFinished = () ->
            m_currentDebouncer.calculate(m_currentSpike.getAsBoolean()) && 
            m_velocityDebouncer.calculate(m_veloIsNearZero.getAsBoolean());

        return new FunctionalCommand(init, execute, onEnd, isFinished, this);
    }

    @Override
    public void periodic() {
        log_topBeamBreak.accept(trg_topBeamBreak);
        log_botBeamBreak.accept(trg_botBeamBreak);
    }
}
