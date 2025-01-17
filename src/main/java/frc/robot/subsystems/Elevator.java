package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;

import static frc.robot.Constants.EleK.*;

/* ALL DUMMY NUMBERS !!! */
/* dummy just like u HA */
public class Elevator {
    private final TalonFX m_left = new TalonFX(0, TunerConstants.kCANBus);
    private final TalonFX m_right = new TalonFX(1, TunerConstants.kCANBus);
    private final Follower m_follower = new Follower(m_right.getDeviceID(), true);  // aka left motor :0
    private final CANcoder m_CANcoder = new CANcoder(1);
    private final DigitalInput m_lowLimit = new DigitalInput(1);
    private final Trigger m_lowLimTrig = new Trigger(m_lowLimit::get).negate();

    private final ProfiledPIDController m_controller = new ProfiledPIDController(1, 0, 1, null);
    private final PIDController m_holdController = new PIDController(0, 0, 0); // make robot stay at a pos

    private double m_targetHeight = 0;
    private double m_dynamicLowLim = 0;
    private double m_pdEffort = 0;
    private double m_ffEffort = 0;

    private double m_holdPdEffort = 0;
    private double m_holdFfEffort = 0;

    private boolean m_isCoast = false;

    private final DoubleLogger
        log_ffEffort, log_pdeEffort, log_totalEffort, log_targetHeight, log_holdPdEffort, log_holdFfEffort,
        log_profileTargetHeight, log_actualHeightMeters, log_actualHeightRaw, log_profileVelo, log_actualVelo;
    private final BooleanLogger log_atLowLimit;

    private final GenericEntry nte_isCoast;

    public Elevator() {
        m_left.getConfigurator().apply(kLeftConfigs);
        m_right.getConfigurator().apply(kRightConfigs);

        m_CANcoder.getConfigurator().apply(kCANcoderConfigs);

        m_left.setNeutralMode(NeutralModeValue.Brake);
        m_right.setNeutralMode(NeutralModeValue.Brake);

        m_left.setControl(m_follower);

        // config velo measurement pd & window

        log_ffEffort = WaltLogger.logDouble("Elevator", "ffEffort");
        log_pdeEffort = WaltLogger.logDouble("Elevator", "ffEffort");
        log_totalEffort = WaltLogger.logDouble("Elevator", "ffEffort");
        log_targetHeight = WaltLogger.logDouble("Elevator", "ffEffort");
        log_holdPdEffort = WaltLogger.logDouble("Elevator", "ffEffort");
        log_holdFfEffort = WaltLogger.logDouble("Elevator", "ffEffort");
        log_profileTargetHeight = WaltLogger.logDouble("Elevator", "ffEffort");
        log_actualHeightMeters = WaltLogger.logDouble("Elevator", "ffEffort");
        log_actualHeightRaw = WaltLogger.logDouble("Elevator", "ffEffort");
        log_profileVelo = WaltLogger.logDouble("Elevator", "ffEffort");
        log_actualVelo = WaltLogger.logDouble("Elevator", "ffEffort");
        log_atLowLimit = WaltLogger.logBoolean("Elevator", "ffEffort");

        nte_isCoast = Shuffleboard.getTab("Elevator")
            .add("coast", false)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .getEntry();

        m_lowLimTrig.onTrue(Commands.runOnce(() -> {
            m_CANcoder.setPosition(0);
        }).ignoringDisable(true));
    }

    /* getters n setters */
    public double getActualHeightRaw() {
        return m_CANcoder.getPosition().getValueAsDouble();
    }

    // TODO: ill finish this later bruh i dont wanna do math rn
    public double getActualHeightMeters() {
        return 1;
    }

    public boolean isFullyRetracted() {
        return !m_lowLimit.get();
    }

    // TODO: ill finish this later bruh i dont wanna do math rn
    private double getActualVelocityMps() {
        return 1;
    }

    /* ele autoretract */
    public Command autoHome() {
        return Commands.startEnd(() -> {
            m_right.setVoltage(-2);
        }, () -> {
            m_right.setVoltage(0);
        }).until(m_lowLimTrig);
    }
}
