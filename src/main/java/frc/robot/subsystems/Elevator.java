package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorK;
import frc.robot.generated.TunerConstants;
import frc.util.WaltLogger;
import frc.util.WaltLogger.DoubleLogger;

//numbers are dummies
public class Elevator extends SubsystemBase {
    private final TalonFX m_right = new TalonFX(ElevatorK.kRightCANID, TunerConstants.kCANBus);
    private final TalonFX m_left = new TalonFX(ElevatorK.kLeftCANID, TunerConstants.kCANBus);
    private final Follower m_follower = new Follower(m_right.getDeviceID(),true);
    private final MotionMagicExpoVoltage m_MMEVRequest = new MotionMagicExpoVoltage(0);

    private final ElevatorSim m_elevatorSim = new ElevatorSim(
            DCMotor.getKrakenX60(2), ElevatorK.kGearRatio, ElevatorK.kCarriageMassKg.in(Kilograms), ElevatorK.kSpoolRadius.in(Meters),
            ElevatorK.kMinimumHeight.in(Meters), ElevatorK.kMaximumHeight.in(Meters), true, ElevatorK.kStartingHeightMeters.in(Meters));
    private final Mechanism2d m_mech2d =
        new Mechanism2d(6, ElevatorK.kMaximumHeight.in(Meters));
    private final MechanismRoot2d m_mech2dRoot =
        m_mech2d.getRoot("Elevator Root", 3, 0.0);
    private final MechanismLigament2d m_elevatorMech2d =
        m_mech2dRoot.append(
            new MechanismLigament2d("Elevator", m_elevatorSim.getPositionMeters(), 90, 6, new Color8Bit(Color.kRed))
        );

    private final DoubleLogger log_rightMotorPosition = WaltLogger.logDouble("Elevator", "rightMotorPosition");
    private final DoubleLogger log_leftMotorPosition = WaltLogger.logDouble("Elevator", "leftMotorPosition");
    private final DoubleLogger log_rightMotorRealVoltage = WaltLogger.logDouble("Elevator", "rightMotorRealVoltage");
    private final DoubleLogger log_leftMotorRealVoltage = WaltLogger.logDouble("Elevator", "leftMotorRealVoltage");
    private final DoubleLogger log_rightMotorSimVoltage = WaltLogger.logDouble("Elevator", "rightMotorSimVoltage");
    private final DoubleLogger log_elevatorSimPosition = WaltLogger.logDouble("Elevator", "simPosition");

    public Elevator() {
        m_left.setControl(m_follower);
        m_left.getConfigurator().apply(ElevatorK.kLeftTalonFXConfiguration);
        m_right.getConfigurator().apply(ElevatorK.kRightTalonFXConfiguration);
        SmartDashboard.putData("Elevator Sim", m_mech2d);

        register();
    }

    public Command setPosition(double heightMeters) {
        return Commands.runOnce(() -> m_right.setControl(m_MMEVRequest.withPosition(heightMeters)), this);
    }

    public Command setPosition(HeightPosition heightMeters) {
        return setPosition(heightMeters.m_heightMeters);
    }

    public void simulationPeriodic() {
        TalonFXSimState rightSim = m_right.getSimState();
        rightSim.setRawRotorPosition(m_elevatorSim.getPositionMeters() * ElevatorK.kSensorToMechanismRatio);
        rightSim.setRotorVelocity(m_elevatorSim.getVelocityMetersPerSecond() * ElevatorK.kSensorToMechanismRatio);


        m_elevatorSim.setInput(rightSim.getMotorVoltage());

        m_elevatorSim.update(0.020);

        log_elevatorSimPosition.accept(m_elevatorSim.getPositionMeters());
        
        // m_right.setPosition(m_elevatorSim.getPositionMeters());

        m_elevatorMech2d.setLength(m_elevatorSim.getPositionMeters());

        log_rightMotorSimVoltage.accept(rightSim.getMotorVoltage());

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
    }

    public void periodic() {
        log_rightMotorPosition.accept(m_right.getPosition().getValueAsDouble());
        log_leftMotorPosition.accept(m_left.getPosition().getValueAsDouble());
        log_rightMotorRealVoltage.accept(m_right.getMotorVoltage().getValueAsDouble());
        log_leftMotorRealVoltage.accept(m_left.getMotorVoltage().getValueAsDouble());        
    }

    public enum HeightPosition {
        HOME(0),
        CORAL_STATION(1),
        CLIMB_UP(1.5), // this height will move the robot up for climb
        L1(2),
        L2(3),
        CLIMB_DOWN(3.5), //this height will move the robot back down from the cage
        L3(4),
        L4(5);

        public final double m_heightMeters;

        private HeightPosition(double heightMeters){
            m_heightMeters = heightMeters;
        }
    }
}