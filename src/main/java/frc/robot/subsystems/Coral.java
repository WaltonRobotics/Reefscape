package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Coralk;
import frc.robot.generated.TunerConstants;
import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;

public class Coral extends SubsystemBase {
    private final TalonFX m_coralMotor = new TalonFX(Coralk.kCoralMotorCANID, TunerConstants.kCANBus);

    private final DigitalInput topIntakeBeamBreak = new DigitalInput(0);     //TODO: set channels to actual values
    private final DigitalInput botIntakeBeamBreak = new DigitalInput(1);

    private final BooleanSupplier bs_topIntakeBeamBreak = () -> !topIntakeBeamBreak.get();     
    private final BooleanSupplier bs_botIntakeBeamBreak = () -> !botIntakeBeamBreak.get();

    public final Trigger trg_topIntakeBeamBreak = new Trigger(bs_topIntakeBeamBreak);
    public final Trigger trg_botIntakeBeamBreak = new Trigger(bs_botIntakeBeamBreak);

    private CoralState m_coralState;

    // private final DoubleLogger log_coralMotorSpeed = WaltLogger.logDouble("Coral", "coralMotorPosition");
    // private final DoubleLogger log_coralMotorRealVoltage = WaltLogger.logDouble("Coral", "coralMotorRealVoltage");

    public Coral() {
        m_coralMotor.getConfigurator().apply(Coralk.kCoralMotorTalonFXConfiguration);

        register();
        configureTriggers();
    }

    private void configureTriggers() {
        // trg_topIntakeBeamBreak.onTrue(m_coralState.);
        trg_topIntakeBeamBreak.whileTrue(setMotorAction(1.0));
        trg_topIntakeBeamBreak.whileFalse(setMotorAction(0));
        trg_botIntakeBeamBreak.whileTrue(setMotorAction(-1.0));
        trg_botIntakeBeamBreak.whileFalse(setMotorAction(0));
    }

    /**
     * @param speed [-1.0, 1.0] (-1 being max outake, 1 being max intake)
     * @return command that tells the motor to set to provided speed
     */
    public Command setSpeed(double speed) {
        return Commands.runOnce(() -> m_coralMotor.set(speed));
    }

    /**
     * @param destinationVelocity Units in percent max velocity [-1.0, 1.0]
     * @return A Command which sets the intake to go to the specificed velocity
     */
    public Command setMotorAction(double destinationVelocity) {
        return Commands.runEnd(
            () -> m_coralMotor.set(destinationVelocity),
            () -> m_coralMotor.set(0)
        );
    }

    public enum MotorSpeed {
        MAX_OUTAKE(-1.0),
        STOP(0.0),
        MAX_INTAKE(1.0);

        public final double m_motorSpeed;

        private MotorSpeed(double motorSpeed) {
            m_motorSpeed = motorSpeed;
        }
    }

    public enum CoralState {
        INTAKING(0),
        INTAKED(1),
        OUTAKING(2),
        OUTAKED(3);

        public final int m_idx;

        private CoralState(int idx) {
            m_idx = idx;
        }
    }
}
