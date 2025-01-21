package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevatork;
import frc.robot.generated.TunerConstants;
import frc.util.WaltLogger;
import frc.util.WaltLogger.DoubleLogger;

//numbers are dummies
public class Elevator extends SubsystemBase{
    private final TalonFX m_left = new TalonFX(Elevatork.kLeftCANID, TunerConstants.kCANBus);
	private final TalonFX m_right = new TalonFX(Elevatork.kRightCANID, TunerConstants.kCANBus);
    private final Follower m_follower = new Follower(m_right.getDeviceID(),true);
    private final MotionMagicExpoVoltage m_MMEVRequest = new MotionMagicExpoVoltage(0);

    private final TalonFXSimState m_rightSim = m_right.getSimState();
    private final ElevatorSim m_elevatorSim = new ElevatorSim(
        DCMotor.getKrakenX60(2), Elevatork.kGearRatio, Elevatork.kCarriageMassKg, Elevatork.kSpoolDiameter,
        0.0, Elevatork.kMaximumHeight, true, Elevatork.kStartingHeightMeters);

    private final DoubleLogger log_elevatorSimPosition = WaltLogger.logDouble("Elevator", "simPosition");

    public Elevator(){
        m_left.setControl(m_follower);
        m_left.getConfigurator().apply(Elevatork.kLeftTalonFXConfiguration);
        m_right.getConfigurator().apply(Elevatork.kRightTalonFXConfiguration);

    }


    public Command setPosition(double heightMeters){
        return Commands.run(() -> m_right.setControl(m_MMEVRequest.withPosition(heightMeters*Elevatork.kMeterstoRotations)));
    }

    public Command setPosition(HeightPosition heightMeters){
        return setPosition(heightMeters.m_heightMeters);
    }

    public void simulationPeriodic(){
        TalonFXSimState rightSim = m_right.getSimState();

        m_elevatorSim.setInput(rightSim.getMotorVoltage());

        m_elevatorSim.update(0.020);

        log_elevatorSimPosition.accept(m_elevatorSim.getPositionMeters());

        m_rightSim.setRawRotorPosition(m_elevatorSim.getPositionMeters() * 1 / Elevatork.kSensorToMechanismRatio);

         RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
    }

    

    public enum HeightPosition{
        HOME(2),
        L1(5),
        L2(6),
        L3(8),
        L4(10);

        public final double m_heightMeters;

        private HeightPosition(double heightMeters){
            m_heightMeters = heightMeters;
        }

    }

}