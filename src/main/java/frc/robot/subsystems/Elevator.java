package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.ElevatorK.*;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import frc.robot.Constants.ElevatorK;
import frc.robot.generated.TunerConstants;
import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;

//numbers are dummies
public class Elevator extends SubsystemBase {
    private final TalonFX m_frontMotor = new TalonFX(kFrontCANID, TunerConstants.kCANBus);
    private final TalonFX m_rearMotor = new TalonFX(kBackCANID, TunerConstants.kCANBus);
    private final Follower m_followerReq = new Follower(m_frontMotor.getDeviceID(),true);
    private MotionMagicExpoVoltage m_MMEVRequest = new MotionMagicExpoVoltage(0);

    private double m_desiredHeight = 0;
    private boolean m_isHomed = false;
    private Debouncer m_debouncer = new Debouncer(0.125, DebounceType.kRising);
    private BooleanSupplier m_currentSpike = () -> m_frontMotor.getStatorCurrent().getValueAsDouble() > 15.0; 
    private VoltageOut zeroingVoltageCtrlReq = new VoltageOut(-1);


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
        m_frontMotor.getConfigurator().apply(kFrontTalonFXConfig);

        m_rearMotor.getConfigurator().apply(kRearTalonFXConfig);
        m_rearMotor.setControl(m_followerReq);

        SmartDashboard.putData("Elevator Sim", m_mech2d);

        setDefaultCommand(currentSenseHoming());
    }

    public boolean nearSetpoint() {
        return nearSetpoint(kTolerancePulleyRotations);
    }

    public boolean nearSetpoint(double tolerancePulleyRotations) {
        double diff = m_MMEVRequest.Position - getPulleyRotations();
        return Math.abs(diff) <= tolerancePulleyRotations;
    }

    private double getPulleyRotations() {
        return m_rearMotor.getPosition().getValueAsDouble();
    }

    private Distance getPositionMeters() {
        return ElevatorK.rotationsToMeters(m_rearMotor.getPosition().getValue());
    }

    /* 
     * use for scoring
     */
    public Command toHeight(EleHeight heightMeters) {
        return toHeight(heightMeters.meters);
    }

    public Command toHeight(double heightMeters) {
        m_desiredHeight = heightMeters;
        double heightRots = ElevatorK.metersToRotation(Meters.of(heightMeters)).in(Rotations);
        return runOnce(
            () -> {
                m_MMEVRequest = m_MMEVRequest.withPosition(heightRots);
                log_elevatorDesiredPosition.accept(Meters.of(heightMeters).magnitude());
                m_rearMotor.setControl(m_MMEVRequest);
            }
        ).until(() -> nearSetpoint());
    }

    public Command overrideToHeight(double input) {
        if(input > 0) {
            return Commands.sequence(
                Commands.runOnce(() -> m_desiredHeight += Meters.of(Units.inchesToMeters(2)).magnitude()), // logic taken from Shosty's increaseAngle() method in Aim
                toHeight(m_desiredHeight)
            );
        } else if(input < 0) {
            return Commands.sequence(
                Commands.runOnce(() -> m_desiredHeight -= Meters.of(Units.inchesToMeters(2)).magnitude()), // logic taken from Shosty's decreaseAngle() method in Aim
                toHeight(m_desiredHeight)
            );
        } else { return Commands.none();}
    }

    public Command testVoltageControl(DoubleSupplier stick) {
        return runEnd(() -> {
            m_frontMotor.setControl(zeroingVoltageCtrlReq.withOutput(-(stick.getAsDouble()) * 6));
        }, () -> {
            m_frontMotor.setControl(zeroingVoltageCtrlReq.withOutput(0));
        }
        );
    }

    public Command currentSenseHoming() {
        Runnable init = () -> {
            m_frontMotor.setControl(zeroingVoltageCtrlReq);
        };
        Runnable execute = () -> {};
        Consumer<Boolean> onEnd = (Boolean interrupted) -> {
            m_frontMotor.setPosition(0);
            m_frontMotor.setControl(zeroingVoltageCtrlReq.withOutput(0));
            removeDefaultCommand();
            m_isHomed = true;
            System.out.println("Zeroed Elevator!!!");
        };

        BooleanSupplier isFinished = () ->
            m_debouncer.calculate(m_currentSpike.getAsBoolean());

        return new FunctionalCommand(init, execute, onEnd, isFinished, this);
    }
    
    public boolean getIsHomed() {
        return m_isHomed;
    }

    @Override
    public void periodic() {
        log_eleAtHeight.accept(nearSetpoint());
    }

    @Override
    public void simulationPeriodic() {
        TalonFXSimState frontSim = m_frontMotor.getSimState();
        m_elevatorSim.setInput(frontSim.getMotorVoltage());

        m_elevatorSim.update(0.020);

        log_elevatorSimPosition.accept(m_elevatorSim.getPositionMeters());
        var elevatorVelocity = 
            metersToRotationVel(m_elevatorSim.getVelocityMetersPerSecond()* kGearRatio);

        frontSim.setRawRotorPosition(m_elevatorSim.getPositionMeters() * kGearRatio);
        frontSim.setRotorVelocity(elevatorVelocity);

        m_elevatorMech2d.setLength(m_elevatorSim.getPositionMeters());
    }


    //all these values here are still not 100% exact (CLIMB_UP and CLIMB_DOWN ARE STILL DUMMY VALUES) and will need tweaking
    public enum EleHeight {
        HOME(Units.inchesToMeters(14.542)),
        L1(Units.inchesToMeters(37)),
        L2(Units.inchesToMeters(48.041)),
        L3(Units.inchesToMeters(64)),
        L4(Units.inchesToMeters(86)),
        CLIMB_UP(1.5), // this height will move the robot up for climb
        CLIMB_DOWN(5), //this height will ove robot down for climb
        HP(Units.inchesToMeters(36)); //human player station intake height

        public final double meters;

       private EleHeight(double heightMeters){
            this.meters = heightMeters;
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