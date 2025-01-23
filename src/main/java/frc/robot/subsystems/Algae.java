package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AlgaeK;

public class Algae {
    private final TalonFX m_wristMotor = new TalonFX(AlgaeK.kWristCANID);
    private final TalonFX m_intakeMotor = new TalonFX(AlgaeK.kIntakeCANID);

    private final MotionMagicExpoVoltage m_MMEVRequest = new MotionMagicExpoVoltage(0);

    public Algae() {
        m_wristMotor.getConfigurator().apply(AlgaeK.kWristConfiguration);
        m_intakeMotor.getConfigurator().apply(AlgaeK.kIntakeConfiguration);

        // no idea if these are redundant but better safe than sorry
        m_wristMotor.setPosition(0);
        m_intakeMotor.setPosition(0);
    }

    public Command setWristPosition(Angle destination) {
        return Commands.runOnce(() -> m_wristMotor.setControl(
            m_MMEVRequest.withPosition(destination)
        ));
    }

    public Command setWristPosition(WristPosition destination) {
        return setWristPosition(destination.m_angle);
    }

    /**
     * @param destinationVelocity Units in percent max velocity [-1.0, 1.0]
     * @return A Command which sets the intake to go to the specificed velocity
     */
    public Command setIntakeAction(double destinationVelocity) {
        return Commands.runEnd(
            () -> m_intakeMotor.set(destinationVelocity),
            () -> m_intakeMotor.set(0)
        );
    }

    public Command setIntakeAction(IntakeSpeed destinationVelocity) {
        return setIntakeAction(destinationVelocity.m_intakeSpeed);
    }

    /**
     * @return A Command which sets the intake to go to 0 speed
     */
    public Command stopIntake() {
        return setIntakeAction(IntakeSpeed.STOP);
    }

    /**
     * An enum that holds the most significant positions for the wrist 
     */
    public enum WristPosition {
        HOME(Degrees.of(0)),
        INTAKE(Degrees.of(-45)),
        PROCESSOR_SHOOT(Degrees.of(5));

        public final Angle m_angle;

        private WristPosition(Angle angle) {
            m_angle = angle;
        }
    }

    /**
     * An enum that holds the most important speeds for intake/outake operation.
     * Measured in percent maximum velocity. (I'm not sorry)
     */
    public enum IntakeSpeed {
        // DUMMY NUMBERS
        STOP(0),
        INTAKE(0.5),
        PROCESSOR_SHOOT(-0.5);

        public final double m_intakeSpeed;

        private IntakeSpeed(double intakeSpeed) {
            m_intakeSpeed = intakeSpeed;
        }
    }
}