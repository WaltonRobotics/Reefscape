package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ElevatorK.*;

import java.util.function.BooleanSupplier;

import frc.robot.generated.TunerConstants;
import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;

//numbers are dummies
public class Elevator extends SubsystemBase {
    private final TalonFX m_right = new TalonFX(kRightCANID, TunerConstants.kCANBus);
    private final TalonFX m_left = new TalonFX(kLeftCANID, TunerConstants.kCANBus);
    private final Follower m_follower = new Follower(m_right.getDeviceID(),true);
    // idk why this is PositionVoltage and not MMEV, but Xandra said she got if off Banks' ele code, so Imma just leave it here for now...
    private final PositionVoltage m_MMEVRequest = new PositionVoltage(0);

    private boolean m_atHeight = false;

    public BooleanSupplier bs_atHeight = () -> m_atHeight;

    private final ElevatorSim m_elevatorSim = new ElevatorSim(
            DCMotor.getKrakenX60(2), 
            kGearRatio, 
            kCarriageMassKg.in(Kilograms), 
            kSpoolRadius.in(Meters),
            kMinimumHeight.in(Meters), 
            kMaximumHeight.in(Meters), 
            false, 
            kStartingHeightMeters.in(Meters));

    private final Mechanism2d m_mech2d =
        new Mechanism2d(6, kMaximumHeight.in(Meters));
    private final MechanismRoot2d m_mech2dRoot =
        m_mech2d.getRoot("Elevator Root", 3, 0.0);
    private final MechanismLigament2d m_elevatorMech2d =
        m_mech2dRoot.append(
            new MechanismLigament2d("Elevator", m_elevatorSim.getPositionMeters(), 90, 6, new Color8Bit(Color.kRed))
        );

    private final DoubleLogger log_elevatorDesiredPosition = WaltLogger.logDouble(kLogTab, "desiredPosition");
    private final DoubleLogger log_elevatorSimPosition = WaltLogger.logDouble(kLogTab, "simPosition");

    private final BooleanLogger log_eleAtHeight = WaltLogger.logBoolean(kLogTab, "atDesiredHeight");

    public Elevator() {
        m_left.setControl(m_follower);
        m_left.getConfigurator().apply(kLeftTalonFXConfiguration);
        m_right.getConfigurator().apply(kRightTalonFXConfiguration);
        SmartDashboard.putData("Elevator Sim", m_mech2d);
    }

    public void invertAtHeight() {
        m_atHeight = !m_atHeight;
    }

    public Command setPosition(double heightMeters) {
        return runOnce(
            () -> { 
                log_elevatorDesiredPosition.accept(heightMeters);
                m_right.setControl(m_MMEVRequest.withPosition(heightMeters));
        });
    }

    // also use for climbing
    public Command toHome() {
        return runOnce(
            () -> { 
                log_elevatorDesiredPosition.accept(0.0);
                m_right.setControl(m_MMEVRequest.withPosition(0));
            }
        );
    }

    public Command toCS() {
        return runOnce(
            () -> {
                log_elevatorDesiredPosition.accept(1.0);
                m_right.setControl(m_MMEVRequest.withPosition(1));
            }
        );
    }

    /* 
     * use for scoring
     */
    public Command setPosition(EleHeights heightMeters) {
        m_atHeight = false;
        return runOnce(
            () -> { 
                log_elevatorDesiredPosition.accept(heightMeters.m_heightMeters);
                m_right.setControl(m_MMEVRequest.withPosition(heightMeters.m_heightMeters));}
        ).andThen(() -> invertAtHeight());
    }

    @Override
    public void periodic() {
        log_eleAtHeight.accept(bs_atHeight);
    }

    @Override
    public void simulationPeriodic() {
        TalonFXSimState rightSim = m_right.getSimState();
        m_elevatorSim.setInput(rightSim.getMotorVoltage());

        m_elevatorSim.update(0.020);

        log_elevatorSimPosition.accept(m_elevatorSim.getPositionMeters());
        var elevatorVelocity = 
            metersToRotationVel(m_elevatorSim.getVelocityMetersPerSecond()* kGearRatio);
        log_eleAtHeight.accept(bs_atHeight);

        rightSim.setRawRotorPosition(m_elevatorSim.getPositionMeters() * kGearRatio);
        rightSim.setRotorVelocity(elevatorVelocity);

        m_elevatorMech2d.setLength(m_elevatorSim.getPositionMeters());
    }


    //all these values here are still not 100% exact (CLIMB_UP and CLIMB_DOWN ARE STILL DUMMY VALUES) and will need tweaking
    public enum EleHeights {
        HOME(Units.inchesToMeters(14.542)),
        L1(Units.inchesToMeters(37)),
        L2(Units.inchesToMeters(48.041)),
        L3(Units.inchesToMeters(64)),
        L4(Units.inchesToMeters(86)),
        CLIMB_UP(1.5), // this height will move the robot up for climb
        CLIMB_DOWN(5); //this height will ove robot down for climb

        public final double m_heightMeters;

       private EleHeights(double heightMeters){
            m_heightMeters = heightMeters;
        }
    }

    public enum AlgaeHeight {
        L2(Units.inchesToMeters(34.782)),
        L3(Units.inchesToMeters(49.235));

        public final double m_heightMeters;

        private AlgaeHeight(double heightMeters){
            m_heightMeters = heightMeters;
        }
    }
}