package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
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
    private final TalonFX m_coralMotor = new TalonFX(kCoralMotorCANID, TunerConstants.kCANBus);

    private final TalonFXS m_fingerMotor = new TalonFXS(kFingerMotorCANID);
    private PositionVoltage m_PosVoltReq = new PositionVoltage(0);

    private boolean m_coralIsRunning = false;

    private boolean m_coralIsCoast = false;
    private GenericEntry nte_coralIsCoast;
    private boolean m_fingerIsCoast = false;
    private GenericEntry nte_fingerIsCoast;

    // true when beam break brokey
    public DigitalInput m_topBeamBreak = new DigitalInput(kTopBeamBreakChannel);
    public DigitalInput m_botBeamBreak = new DigitalInput(kBotBeamBreakChannel);

    public final BooleanSupplier bs_topBeamBreak = () -> m_topBeamBreak.get();
    public final BooleanSupplier bs_botBeamBreak = () -> m_botBeamBreak.get();

    public final BooleanSupplier bs_coralMotorRunning = () -> m_coralIsRunning;

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

    public void setCoralCoast(boolean coast) {
        m_coralMotor.setNeutralMode(coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
    }

    public void setFingerCoast(boolean coast) {
        m_fingerMotor.setNeutralMode(coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
    }

    private void falsifyCoralIsRunning() {
        m_coralIsRunning = false;
    }

    /**
     * @param destinationVelocity Units in percent max velocity [-1.0, 1.0]
     * @return A Command which sets the intake to go to the specificed velocity
     */
    public Command setCoralMotorAction(double destinationVelocity) {
        m_coralIsRunning = true;
        return Commands.runEnd(
            () -> m_coralMotor.set(destinationVelocity),
            () -> m_coralMotor.set(0)
        ).andThen(() -> falsifyCoralIsRunning());
    }

    public Command intake() {
        return setCoralMotorAction(1).until(bs_botBeamBreak);
    }

    public Command score() {
        return setCoralMotorAction(-1).until(() -> bs_botBeamBreak.getAsBoolean() == false);
    }

    public Command fingerOut() {
        return runOnce(
            () -> m_fingerMotor.setControl(m_PosVoltReq.withPosition(Angle.ofRelativeUnits(180, Degrees))));
    }

    public Command fingerIn() {
        return runOnce(
            () -> m_fingerMotor.setControl(m_PosVoltReq.withPosition(Angle.ofRelativeUnits(0, Degrees))));
    }

    @Override
    public void periodic() {
        m_coralIsCoast = nte_coralIsCoast.getBoolean(false);
        setCoralCoast(m_coralIsCoast);
        m_fingerIsCoast = nte_fingerIsCoast.getBoolean(false);
        setFingerCoast(m_fingerIsCoast);

        log_topBeamBreak.accept(bs_topBeamBreak);
        log_botBeamBreak.accept(bs_botBeamBreak);
    }
}
