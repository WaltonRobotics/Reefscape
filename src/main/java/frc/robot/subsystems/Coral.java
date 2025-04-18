package frc.robot.subsystems;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.Coralk.*;

import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;

public class Coral extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(kCoralMotorCANID);
    private VoltageOut m_voltOutReq = new VoltageOut(0);
    private NeutralOut m_neutralOut = new NeutralOut();

    private final double m_slowIntakeSpeed = 3;
    private final double m_scoreSpeed = 4.5;

    // true when beam break broken
    public DigitalInput m_topBeamBreak = new DigitalInput(kTopBeamBreakChannel);
    public DigitalInput m_botBeamBreak = new DigitalInput(kBotBeamBreakChannel);

    public final Trigger trg_topBeamBreak = new Trigger(() -> !m_topBeamBreak.get());
    public final Trigger trg_botBeamBreak = new Trigger(() -> !m_botBeamBreak.get());

    private final BooleanLogger log_topBeamBreak = WaltLogger.logBoolean(kLogTab, "topBeamBreak");
    private final BooleanLogger log_botBeamBreak = WaltLogger.logBoolean(kLogTab, "botBeamBreak");

    public Coral() {
        m_motor.getConfigurator().apply(kCoralMotorTalonFXConfiguration);
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
        m_motor.setNeutralMode(coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
    }

    private void setCoralMotorAction(double voltage) {
        m_motor.setControl(m_voltOutReq.withOutput(voltage));
    }

    public Command setCoralMotorActionCmd(double voltage) {
        return runOnce(() -> setCoralMotorAction(voltage));
    }

    public void stopCoralMotor() {
        m_motor.setControl(m_neutralOut);
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
        return setCoralMotorActionCmd(-m_scoreSpeed);
    }

    public Command score() {
        return setCoralMotorActionCmd(m_scoreSpeed);
    }

    public void runWheelsAlgaeRemoval() {
        setCoralMotorAction(-m_scoreSpeed);
    }

    @Override
    public void periodic() {
        log_topBeamBreak.accept(trg_topBeamBreak);
        log_botBeamBreak.accept(trg_botBeamBreak);
    }
}
