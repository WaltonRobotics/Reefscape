package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;

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

    private BooleanLogger log_fingerOut = WaltLogger.logBoolean(kLogTab, "finger out");
    
    private boolean m_isHomed = false;

    public Finger() {
        m_motor.getConfigurator().apply(kTalonFXSConfig);

        setDefaultCommand(currentSenseHoming());
    }

    private void setFingerPos(double rotations) {
        m_motor.setControl(m_PosVoltReq.withPosition(rotations));
    }

    public void fingerOut() {
        log_fingerOut.accept(true);
        System.out.println("Finger Requested out");
        m_motor.setControl(m_PosVoltReq.withPosition(kParallelToGroundRotations));
    }

    public Command fingerOutCmd() {
        return runOnce(this::fingerOut);
    }

    public Command fingerPrepareForClimbCmd() {
        return runOnce(() -> setFingerPos(kClimbRotations)); // idk if the climb rots r right. we gotta test this
    }

    public Command fingerPrepareForIntakeCmd() {
        return runOnce(() -> setFingerPos(kIntakeRotations));
    }

    public void fingerIn() {
        log_fingerOut.accept(false);
        System.out.println("Finger Requested In");
        m_motor.setControl(m_PosVoltReq.withPosition(kDefaultPos));
    }

    public Command fingerInCmd() {
        return runOnce(this::fingerIn);
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
}
