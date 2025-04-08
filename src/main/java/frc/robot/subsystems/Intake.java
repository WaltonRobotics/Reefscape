package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.IntakeK.*;

import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;

//TODO: actually finish when recieve basic plan
public class Intake extends SubsystemBase{

    private final TalonFX m_motor = new TalonFX(kIntakeMotorCANID);
    private VoltageOut m_voltOutReq = new VoltageOut(0);
    private NeutralOut m_neutralOut = new NeutralOut();

    public DigitalInput m_intakeBeamBreak = new DigitalInput(kBeamBreakChannel);

    public final Trigger trg_intakeBeamBreak = new Trigger(() -> !m_intakeBeamBreak.get());

    private final BooleanLogger log_intakeBeamBreak = WaltLogger.logBoolean(kLogTab, "intakeBeamBreak");
    
    public Intake() {
        m_motor.getConfigurator().apply(kIntakeConfiguration);

        setDefaultCommand(setBrakeCommand());
    }

    public Command automaticIntake() {
        return Commands.startEnd(() -> fastIntake(),() -> stopIntakeMotorCmd());
    }

    public void setBrake() {
        m_motor.setControl(new StaticBrake());
    }

    public Command setBrakeCommand() {
        return runOnce(() -> setBrake());
    }

    public void setIntakeCoast(boolean coast) {
        m_motor.setNeutralMode(coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
    }

    public Command setIntakeCoastCommand(boolean coast) {
        return runOnce(() -> setIntakeCoast(coast));
    }

    private void setIntakeMotorAction(double voltage) {
        m_motor.setControl(m_voltOutReq.withOutput(voltage));
    }

    public Command setIntakeMotorActionCmd(double voltage) {
        return runOnce(() -> setIntakeMotorAction(voltage));
    }

    public void stopIntakeMotor() {
        m_motor.setControl(m_neutralOut);
    }
    
    public Command stopIntakeMotorCmd() {
        return runOnce(this::stopIntakeMotor);
    }

    //double check voltage value
    public Command fastIntake() {
        return setIntakeMotorActionCmd(12);
    }
    @Override
    public void periodic() {
        log_intakeBeamBreak.accept(trg_intakeBeamBreak);
    }
}
