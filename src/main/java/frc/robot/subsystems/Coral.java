package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Coralk;
import frc.robot.generated.TunerConstants;
import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;

import static frc.robot.subsystems.Coral.CoralState.*;

public class Coral extends SubsystemBase {
    private final TalonFX m_coralMotor = new TalonFX(Coralk.kCoralMotorCANID, TunerConstants.kCANBus);

    private final DigitalInput topBeamBreak = new DigitalInput(0);     //TODO: set channels to actual values
    private final DigitalInput botBeamBreak = new DigitalInput(1);

    public final EventLoop sensorEventLoop = new EventLoop();
    public final EventLoop stateEventLoop = new EventLoop();

    private CoralState m_coralState;

    // state triggers
    public final Trigger stateTrg_noCoral = new Trigger(stateEventLoop, () -> m_coralState == NO_CORAL);
    public final Trigger stateTrg_intaking = new Trigger(stateEventLoop, () -> m_coralState == INTAKING);
    public final Trigger stateTrg_intook = new Trigger(stateEventLoop, () -> m_coralState == INTOOK);
    public final Trigger stateTrg_elevator = new Trigger(stateEventLoop, () -> m_coralState == ELEVATOR);
    public final Trigger stateTrg_scoreReady = new Trigger(stateEventLoop, () -> m_coralState == SCORE_READY);
    public final Trigger stateTrg_scoring = new Trigger(stateEventLoop, () -> m_coralState == SCORING);
    
    // beam break triggers
    private boolean topBeamBreakIrq = false;
    private boolean botBeamBreakIrq = false;

    public final Trigger irqTrg_topBeamBreak = new Trigger(sensorEventLoop, () -> topBeamBreakIrq);
    public final Trigger irqTrg_botBeamBreak = new Trigger(sensorEventLoop, () -> botBeamBreakIrq);

    public Coral() {
        m_coralMotor.getConfigurator().apply(Coralk.kCoralMotorTalonFXConfiguration);

        register();
        configureStateTriggers();
    }

    private Command changeStateCmd(CoralState state) {
        return Commands.runOnce(() -> {
            if (m_coralState == state) { return; }
            var oldState = m_coralState;
            m_coralState = state;
            System.out.println("changing state from " + oldState + " to " + m_coralState);
        }).withName("SuperStateChange_To" + state);
    }

    private void configureStateTriggers() {
        irqTrg_topBeamBreak.onTrue(Commands.runOnce(() -> m_coralState = INTAKING));
        irqTrg_botBeamBreak.onTrue(Commands.runOnce(() -> m_coralState = INTOOK));
        // i think this should be continued in superstructure
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
        NO_CORAL(0),
        INTAKING(1), // STARTS WHEN FIRST ONE BREAKS ENDS WHEN SECOND ONE BREAKS: wheels run
        INTOOK(2), // WHEN SECOND BEAM BREAK BREAKS: wheels stop run
        ELEVATOR(3),
        SCORE_READY(4), //when elevator is at desired height: button pressing is allowed to do something
        SCORING(5); //hitting a button: wheels running

        public final int m_idx;

        private CoralState(int idx) {
            m_idx = idx;
        }
    }
}
