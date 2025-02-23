package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Coralk.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;

public class Coral extends SubsystemBase {
    private final TalonFX m_coralMotor = new TalonFX(kCoralMotorCANID);

    private final TalonFXS m_fingerMotor = new TalonFXS(kFingerMotorCANID);
    private PositionVoltage m_PosVoltReq = new PositionVoltage(0);
    private VoltageOut m_voltOutReq = new VoltageOut(0);

    private final double m_slowIntakeSpeed = 12 * .25;
    private final double m_scoreSpeed = 6;

    private boolean m_coralIsCoast = false;
    private GenericEntry nte_coralIsCoast;
    private boolean m_fingerIsCoast = false;
    private GenericEntry nte_fingerIsCoast;

    // true when beam break brokey
    public DigitalInput m_topBeamBreak = new DigitalInput(kTopBeamBreakChannel);
    public DigitalInput m_botBeamBreak = new DigitalInput(kBotBeamBreakChannel);

    public final BooleanSupplier bs_topBeamBreak = () -> !m_topBeamBreak.get();
    public final BooleanSupplier bs_botBeamBreak = () -> !m_botBeamBreak.get();

    private final BooleanLogger log_topBeamBreak = WaltLogger.logBoolean(kLogTab, "topBeamBreak");
    private final BooleanLogger log_botBeamBreak = WaltLogger.logBoolean(kLogTab, "botBeamBreak");

    public Coral() {
        m_coralMotor.getConfigurator().apply(kCoralMotorTalonFXConfiguration);
        m_fingerMotor.getConfigurator().apply(kFingerMotorTalonFXSConfig);

        nte_coralIsCoast = Shuffleboard.getTab(kLogTab)
                  .add("coral coast", false)
                  .withWidget(BuiltInWidgets.kToggleSwitch)
                  .getEntry();
        nte_fingerIsCoast = Shuffleboard.getTab(kLogTab)
                  .add("finger coast", false)
                  .withWidget(BuiltInWidgets.kToggleSwitch)
                  .getEntry();
    }

    // good method
    public Command automaticCoralIntake() {
        return Commands.sequence(
            fastIntake().until(bs_topBeamBreak),
            slowIntake().until(bs_botBeamBreak),
            stopCoralMotorCmd()
        );
    }

    public void setCoralCoast(boolean coast) {
        m_coralMotor.setNeutralMode(coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
    }

    /**
     * @param destinationVelocity Units in voltage [-12.0, 12.0]
     * @return A Command which sets the intake to go to the specificed velocity
     */
    public Command setCoralMotorAction(double voltage, double endVoltage) {
        return Commands.runEnd(
            () -> m_coralMotor.setControl(m_voltOutReq.withOutput(voltage)),
            () -> m_coralMotor.setControl(m_voltOutReq.withOutput(endVoltage))
        );
    }

    private void setCoralMotorAction(double voltage) {
        m_coralMotor.setControl(m_voltOutReq.withOutput(voltage));
    }

    public Command setCoralMotorActionCmd(double destinationVelocity) {
        return runOnce(() -> setCoralMotorAction(destinationVelocity));
    }

    private void stopCoralMotor() {
        m_coralMotor.setControl(m_voltOutReq.withOutput(0));
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

    @Override
    public void periodic() {
        m_coralIsCoast = nte_coralIsCoast.getBoolean(false);
        m_fingerIsCoast = nte_fingerIsCoast.getBoolean(false);

        // setCoralCoast(m_coralIsCoast);
        // setFingerCoast(m_fingerIsCoast);

        log_topBeamBreak.accept(bs_topBeamBreak);
        log_botBeamBreak.accept(bs_botBeamBreak);
    }
}
