package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.Constants.Coralk.*;

import java.util.function.BooleanSupplier;

import frc.robot.generated.TunerConstants;
import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;

public class Coral extends SubsystemBase {
    private final TalonFX m_coralMotor = new TalonFX(kCoralMotorCANID);

    private final TalonFXS m_fingerMotor = new TalonFXS(kFingerMotorCANID);
    private PositionVoltage m_PosVoltReq = new PositionVoltage(0);
    private VoltageOut m_VoltOutReq = new VoltageOut(0);

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
            stopCoralMotor()
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
            () -> m_coralMotor.setControl(m_VoltOutReq.withOutput(voltage)),
            () -> m_coralMotor.setControl(m_VoltOutReq.withOutput(endVoltage))
        );
    }

    public Command setCoralMotorAction(double destinationVelocity) {
        return setCoralMotorAction(destinationVelocity, 0);
    }

    public Command stopCoralMotor() {
        return runOnce(() -> m_coralMotor.setControl(m_VoltOutReq.withOutput(0)));
    }

    public Command fastIntake() {
        return setCoralMotorAction(12);
    }

    /*
     * This happens right after the Top Beam Break occurs so that we dont *woosh* the coral out
     */
    public Command slowIntake(){
        return setCoralMotorAction(m_slowIntakeSpeed);
    }

    public Command score() {
        return setCoralMotorAction(m_scoreSpeed);
    }

    // finger methods
    public Command fingerOut() {
        return runOnce(
            () -> m_fingerMotor.setControl(m_PosVoltReq.withPosition(Angle.ofRelativeUnits(180, Degrees))));
    }

    public Command fingerIn() {
        return runOnce(
            () -> m_fingerMotor.setControl(m_PosVoltReq.withPosition(Angle.ofRelativeUnits(0, Degrees))));
    }

    public Command runFinger() {
        return setCoralMotorAction(-m_scoreSpeed);
    }

    public void setFingerCoast(boolean coast) {
        m_fingerMotor.setNeutralMode(coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
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
