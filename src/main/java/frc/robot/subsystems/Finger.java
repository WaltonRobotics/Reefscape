package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.FingerK.*;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Finger extends SubsystemBase {
    private final TalonFXS m_motor = new TalonFXS(kFingerMotorCANID);

    private VoltageOut m_fingerZeroingVoltageCtrlReq = new VoltageOut(0);
    private VoltageOut m_voltOutReq = new VoltageOut(0);
    private PositionVoltage m_PosVoltReq = new PositionVoltage(0).withEnableFOC(false);


    private BooleanSupplier m_currentSpike = () -> m_motor.getStatorCurrent().getValueAsDouble() > 5.0; 
    private BooleanSupplier m_veloIsNearZero = () -> Math.abs(m_motor.getVelocity().getValueAsDouble()) < 0.005;

    private Debouncer m_currentDebouncer = new Debouncer(0.25, DebounceType.kRising);
    private Debouncer m_velocityDebouncer = new Debouncer(0.125, DebounceType.kRising);
    
    private boolean m_isHomed = false;
    private double m_desiredAngleRots = 0;

    private BooleanLogger log_isHomed = new WaltLogger.BooleanLogger(kLogTab, "Finger is homed");
    private DoubleLogger log_desiredPos = new WaltLogger.DoubleLogger(kLogTab, "Finger desired pos");
    private DoubleLogger log_actualPos = new WaltLogger.DoubleLogger(kLogTab, "Finger actual pos");

    public Finger() {
        m_motor.getConfigurator().apply(kTalonFXSConfig);
        setDefaultCommand(currentSenseHoming());
    }

    public void setFingerPos(FingerPos fingerPos) {
        m_desiredAngleRots = fingerPos.angleRots;
        m_motor.setControl(m_PosVoltReq.withPosition(m_desiredAngleRots));
    }

    public Command fingerOutCmd() {
        return runOnce(() -> setFingerPos(FingerPos.OUT));
    }

    public Command fingerInCmd() {
        return runOnce(() -> setFingerPos(FingerPos.OUT));
    }

    // this is bad but we're like not gonna have a climber so ykw idc #freedom #majorcope #cryinglowk #riprankingpts
    // im still keeping it tho j in case we climb for grits
    // awh look at me caring abt the team after i graduate im so nice
    public Command fingerPrepareForClimbCmd() {
        return runOnce(() -> setFingerPos(FingerPos.DOWN)); // idk if the climb rots r right. we gotta test this
    }

    public Command testFingerVoltageControl(DoubleSupplier stick) {
        return runEnd(() -> {
            m_motor.setControl(m_voltOutReq.withOutput(-(stick.getAsDouble()) * 6));
        }, () -> {
            m_motor.setControl(m_voltOutReq.withOutput(0));
        }
        );
    }

    public void setFingerCoast(boolean coast) {
        m_motor.setNeutralMode(coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
    }

    public Command currentSenseHoming() {
        Runnable init = () -> {
            m_motor.getConfigurator().apply(kSoftLimitSwitchDisabledConfig);
            m_motor.setControl(m_fingerZeroingVoltageCtrlReq.withOutput(7));
        };
        Runnable execute = () -> {};
        Consumer<Boolean> onEnd = (Boolean interrupted) -> {
            m_motor.setPosition(0);
            m_motor.setControl(m_fingerZeroingVoltageCtrlReq.withOutput(0));
            removeDefaultCommand();
            m_isHomed = true;
            m_motor.getConfigurator().apply(kSoftLimitEnabledConfig);
            System.out.println("Zeroed Finger!!!");
            m_motor.setControl(m_PosVoltReq.withPosition(kDefaultPos));
        };

        BooleanSupplier isFinished = () ->
            m_currentDebouncer.calculate(m_currentSpike.getAsBoolean()) && 
            m_velocityDebouncer.calculate(m_veloIsNearZero.getAsBoolean());

        return new FunctionalCommand(init, execute, onEnd, isFinished, this);
    }

    @Override
    public void periodic() {
        log_isHomed.accept(m_isHomed);
        log_desiredPos.accept(m_desiredAngleRots);
        log_actualPos.accept(m_motor.getPosition().getValue().in(Rotations));
    }

    public enum FingerPos {
        OUT(kParallelToGroundRotations),
        IN(kDefaultPos),
        DOWN(kClimbRotations); // TODO: name better (rip climb)

        public double angleRots;
        private FingerPos(double rots) {
            angleRots = rots;
        }
    }
}
