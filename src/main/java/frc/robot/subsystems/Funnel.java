package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static frc.robot.Constants.FunnelK.*;

import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;

public class Funnel extends SubsystemBase{

    private final TalonFXS m_motor = new TalonFXS(kFunnelMotorCANID);
    private VoltageOut m_voltOutReq = new VoltageOut(0);
    private NeutralOut m_neutralOut = new NeutralOut();

    public DigitalInput m_intakeBeamBreak = new DigitalInput(kBeamBreakChannel);

    // velocity already 50hz
    private final StatusSignal<AngularVelocity> m_velocitySignal = m_motor.getVelocity();
    private final StatusSignal<AngularAcceleration> m_accelSignal = m_motor.getAcceleration();
    private final StatusSignal<Boolean> m_StatorCurrLimitSignal = m_motor.getFault_StatorCurrLimit();
    

    public final Trigger trg_intakeBeamBreak = new Trigger(() -> !m_intakeBeamBreak.get());
    final AngularAcceleration kAccelDropSpeed = RotationsPerSecondPerSecond.of(-200);
    private final Trigger accelDropTrig = new Trigger(() -> m_accelSignal.refresh().getValue().lte(kAccelDropSpeed));
    public final Trigger sawStatorCurrLimit = new Trigger(() -> m_StatorCurrLimitSignal.refresh().getValue());
    public final Trigger trg_atCurrLim = sawStatorCurrLimit.and(accelDropTrig);

    private final BooleanLogger log_intakeBeamBreak = WaltLogger.logBoolean(kLogTab, "intakeBeamBreak");
    private final DoubleLogger log_velo = WaltLogger.logDouble(kLogTab, "rps", PubSubOption.sendAll(true));
    private final DoubleLogger log_accel = WaltLogger.logDouble(kLogTab, "rpsps", PubSubOption.sendAll(true));
    private final BooleanLogger log_accelDrop = WaltLogger.logBoolean(kLogTab, "accelDrop", PubSubOption.sendAll(true));
    private final BooleanLogger log_currSpike = WaltLogger.logBoolean(kLogTab, "currSpike", PubSubOption.sendAll(true));
    private final BooleanLogger log_hasCoral = WaltLogger.logBoolean(kLogTab, "hasCoral");
    
    public Funnel() {
        m_motor.getConfigurator().apply(kFunnelConfig);
        m_StatorCurrLimitSignal.setUpdateFrequency(100);        
    }

    public Command automaticIntake() {
        return startEnd(() -> fast(), () -> stopCmd());
    }

    private void setIntakeMotorAction(double voltage) {
        m_motor.setControl(m_voltOutReq.withOutput(voltage));
    }

    private Command setMotorVoltageCmd(double voltage) {
        return runOnce(() -> setIntakeMotorAction(voltage));
    }

    public void stop() {
        m_motor.setControl(m_neutralOut);
    }
    
    public Command stopCmd() {
        return runOnce(this::stop);
    }

    public Command fast() {
        return setMotorVoltageCmd(12);
    }

    public Command ejectFlap() {
        return startEnd(
            () -> {
                setIntakeMotorAction(-12);
                System.out.println("starting eject flap");
            }, 
            () -> {
                setIntakeMotorAction(0);
                System.out.println("ending eject flap");
            });
    }

    @Override
    public void periodic() {
        log_intakeBeamBreak.accept(trg_intakeBeamBreak);
        log_currSpike.accept(sawStatorCurrLimit);
        log_hasCoral.accept(trg_atCurrLim);
        log_velo.accept(m_velocitySignal.refresh().getValueAsDouble());
        log_accel.accept(m_accelSignal.refresh().getValueAsDouble());
        log_accelDrop.accept(accelDropTrig);
    }
}
