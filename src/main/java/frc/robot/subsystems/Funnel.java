package frc.robot.subsystems;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.FunnelK.*;
import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;

//TODO: actually finish when recieve basic plan
public class Funnel extends SubsystemBase{

    private final TalonFXS m_motor = new TalonFXS(kFunnelMotorCANID);
    private VoltageOut m_voltOutReq = new VoltageOut(0);
    private NeutralOut m_neutralOut = new NeutralOut();

    public DigitalInput m_intakeBeamBreak = new DigitalInput(kBeamBreakChannel);

    public final Trigger trg_intakeBeamBreak = new Trigger(() -> !m_intakeBeamBreak.get());

    private final BooleanLogger log_intakeBeamBreak = WaltLogger.logBoolean(kLogTab, "intakeBeamBreak");
    
    public Funnel() {
        m_motor.getConfigurator().apply(kFunnelConfig);
    }

    public Command automaticIntake() {
        return Commands.startEnd(() -> fast(),() -> stopCmd());
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

    @Override
    public void periodic() {
        log_intakeBeamBreak.accept(trg_intakeBeamBreak);
    }
}
