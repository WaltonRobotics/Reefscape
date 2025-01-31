package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.util.WaltLogger;
import frc.util.WaltLogger.DoubleLogger;

import static frc.robot.Constants.AlgaeK.*;

public class Algae extends SubsystemBase {
    private final TalonFX m_wristMotor = new TalonFX(kWristCANID);
    private final TalonFX m_intakeMotor = new TalonFX(kIntakeCANID);

    private double m_targetAngleDeg = 0;

    private final MotionMagicExpoVoltage m_MMEVRequest = new MotionMagicExpoVoltage(0);

    public final Trigger m_currentDraw = new Trigger(() -> m_intakeMotor.getStatorCurrent().getValueAsDouble() > 20).debounce(.1); // dummy numbers

    private boolean m_WristIsCoast = false;
    private boolean m_IntakeIsCoast = false;

    private final DoubleLogger log_wristDeg = WaltLogger.logDouble(kLogTab, "angle degrees");
    private final DoubleLogger log_wristTargetAngleDeg = WaltLogger.logDouble(kLogTab, "target angle degrees");
    private final DoubleLogger log_intakeCurrentDraw = WaltLogger.logDouble(kLogTab, "intake current draw");
    private final GenericEntry nte_wristIsCoast;
    private final GenericEntry nte_intakeIsCoast;

    public Algae() {
        m_wristMotor.getConfigurator().apply(kWristConfiguration);
        m_intakeMotor.getConfigurator().apply(kIntakeConfiguration);

        m_wristMotor.setNeutralMode(NeutralModeValue.Brake);

        nte_wristIsCoast = Shuffleboard.getTab(kLogTab)
                  .add("wrist coast", false)
                  .withWidget(BuiltInWidgets.kToggleSwitch)
                  .getEntry();
        nte_intakeIsCoast = Shuffleboard.getTab(kLogTab)
        .add("intake coast", false)
        .withWidget(BuiltInWidgets.kToggleSwitch)
        .getEntry();
    }

    public void setCoast(TalonFX motor, boolean coast) {
        motor.setNeutralMode(coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
    }

    /* WRIST */

    public Command toAngle(double destinationDeg) { 
        m_targetAngleDeg = destinationDeg;
        return Commands.runOnce(() -> {
            log_wristTargetAngleDeg.accept(destinationDeg); // is this right? who knows. i dont!
            m_wristMotor.setControl(
                m_MMEVRequest.withPosition(Degrees.of(destinationDeg)));
        });
    }

    public Command toAngle(WristPosition destination) {
        return toAngle(destination.m_angleDeg);
    }

    public Angle getAngle() {
        return Rotations.of(m_intakeMotor.getPosition().getValueAsDouble());
    }

    // taken from Shosty code
    private double getDegrees() {
        return Units.rotationsToDegrees(m_intakeMotor.getPosition().getValueAsDouble()) + 28;
    }

    public double getTargetAngleDeg() {
        return m_targetAngleDeg;
    }

    /* INTAKE */
    /**
     * @param destinationVelocity Percent max velocity [-1.0, 1.0]
     * @return A Command which sets the intake to go to the specificed velocity
     */
    public Command setWheelAction(double destinationVelocity) {
        return Commands.runEnd(
            () -> m_intakeMotor.set(destinationVelocity),
            () -> m_intakeMotor.set(0)
        ).until(() -> m_currentDraw.getAsBoolean());
    }

    /**
     * @param destinationVelocity Choose speed from IntakeSpeed
     * @return A Command which sets the intake to go the specified velocity
     */
    public Command setWheelAction(IntakeSpeed destinationVelocity) {
        return setWheelAction(destinationVelocity.m_intakeSpeed);
    }

    @Override
    public void periodic() {
        log_wristDeg.accept(getDegrees());
        log_wristTargetAngleDeg.accept(getTargetAngleDeg());
        log_intakeCurrentDraw.accept(m_intakeMotor.getStatorCurrent().getValueAsDouble());
        
        m_WristIsCoast = nte_wristIsCoast.getBoolean(false);
        m_IntakeIsCoast = nte_intakeIsCoast.getBoolean(false);
        setCoast(m_wristMotor, m_WristIsCoast);
        setCoast(m_intakeMotor, m_IntakeIsCoast);
    }

    /**
     * An enum that holds the most significant positions for the wrist 
     * 
     * these values stil aren't super accurate yet and still need testing
     */
    public enum WristPosition {
        HOME(Degrees.toBaseUnits(41.896)),
        INTAKE(Degrees.toBaseUnits(163)),
        PROCESSOR_SHOOT(Degrees.toBaseUnits(90));

        public final double m_angleDeg;

        private WristPosition(double angle) {
            m_angleDeg = angle;
        }
    }

    /**
     * An enum that holds the most important speeds for intake/outake operation.
     * Measured in percent maximum velocity
     */
    public enum IntakeSpeed {
        // DUMMY NUMBERS
        INTAKE(0.5),
        PROCESSOR_SHOOT(-0.5);

        public final double m_intakeSpeed;

        private IntakeSpeed(double intakeSpeed) {
            m_intakeSpeed = intakeSpeed;
        }
    }
}