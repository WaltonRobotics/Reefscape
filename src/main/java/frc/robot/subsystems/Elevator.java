package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static frc.robot.Constants.ElevatorK.*;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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
    private MotionMagicVoltage m_MMVRequest = new MotionMagicVoltage(0).withEnableFOC(true);

    private double m_desiredHeight = 0; // needs to be logged
    private boolean m_isHomed = false;
    private Debouncer m_currentDebouncer = new Debouncer(0.125, DebounceType.kRising);
    private Debouncer m_velocityDebouncer = new Debouncer(0.125, DebounceType.kRising);
    private BooleanSupplier m_currentSpike = () -> m_frontMotor.getStatorCurrent().getValueAsDouble() > 35.0; 
    private BooleanSupplier m_veloIsNearZero = () -> Math.abs(m_frontMotor.getVelocity().getValueAsDouble()) < 0.01;
    private VoltageOut m_voltageCtrlReq = new VoltageOut(0);

    private boolean m_isCoast = false;
    private GenericEntry nte_coast = Shuffleboard.getTab(kLogTab)
        .add("Coast", false)
        .withWidget(BuiltInWidgets.kToggleSwitch)
        .getEntry();


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
    private final DoubleLogger log_elevatorActualMeters = WaltLogger.logDouble(kLogTab, "actualHeightMeters");

    /* SysId routine for characterizing linear motion. This is used to find PID gains for the elevator. */
    private final SysIdRoutine m_sysIdRoutineLinear = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdLinear_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> m_frontMotor.setControl(m_voltageCtrlReq.withOutput(output)),
            null,
            this
        )
    );

    /* The SysId routine to test */
    private final SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineLinear;

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    public Elevator() {
        m_frontMotor.getConfigurator().apply(kFrontTalonFXConfig);

        m_rearMotor.getConfigurator().apply(kRearTalonFXConfig);
        m_rearMotor.setControl(m_followerReq);

        SmartDashboard.putData("Elevator Sim", m_mech2d);

        setDefaultCommand(currentSenseHoming());

    }

    private void setCoast(boolean coast) {
        if (m_isCoast != coast) {
            m_isCoast = coast;
            m_frontMotor.setNeutralMode(coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
            m_rearMotor.setNeutralMode(coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
        }
    }

    public boolean nearSetpoint() {
        return nearSetpoint(kTolerancePulleyRotations);
    }

    public boolean nearSetpoint(double tolerancePulleyRotations) {
        double diff = m_MMVRequest.Position - getPulleyRotations();
        return Math.abs(diff) <= tolerancePulleyRotations;
    }

    private double getPulleyRotations() {
        return m_frontMotor.getPosition().getValueAsDouble();
    }

    private Distance getPositionMeters() {
        return ElevatorK.rotationsToMeters(m_frontMotor.getPosition().getValue());
    }

    /* 
     * use for scoring
     */
    public Command toHeightCoral(Supplier<EleHeight> height) {
        return toHeight(height.get().rotations);
    }

    public Command toHeightAlgae(Supplier<AlgaeHeight> height) {
        return toHeight(height.get().rotations);
    }

    public Command toHeight(double rotations) {
        return runOnce(
            () -> {
                m_desiredHeight = rotations;
                // double heightRots = ElevatorK.metersToRotation(Meters.of(rotations)).in(Rotations);
                m_MMVRequest = m_MMVRequest.withPosition(rotations);
                log_elevatorDesiredPosition.accept(rotations);
                m_frontMotor.setControl(m_MMVRequest);
            }
        ).until(() -> nearSetpoint());
    }

    public Command testVoltageControl(DoubleSupplier stick) {
        return runEnd(() -> {
            m_frontMotor.setControl(m_voltageCtrlReq.withOutput(-(stick.getAsDouble()) * 6));
        }, () -> {
            m_frontMotor.setControl(m_voltageCtrlReq.withOutput(0));
        }
        );
    }

    public Command currentSenseHoming() {
        Runnable init = () -> {
            System.out.println("Elevator Zero INIT");
            m_frontMotor.getConfigurator().apply(kSoftLimitSwitchDisabledConfig);
            m_frontMotor.setControl(m_voltageCtrlReq.withOutput(-1));
        };
        Runnable execute = () -> {};
        Consumer<Boolean> onEnd = (Boolean interrupted) -> {
            if (interrupted) {
                System.out.println("Elevator homing INTERRUPTED!");
                return;
            }
            m_frontMotor.setPosition(0);
            m_frontMotor.setControl(m_voltageCtrlReq.withOutput(0));
            removeDefaultCommand();
            m_isHomed = true;
            m_frontMotor.getConfigurator().apply(kSoftwareLimitConfigs);
            System.out.println("Zeroed Elevator!!!");
        };

        BooleanSupplier isFinished = () ->
            m_currentDebouncer.calculate(m_currentSpike.getAsBoolean()) && 
            m_velocityDebouncer.calculate(m_veloIsNearZero.getAsBoolean());

        return new FunctionalCommand(init, execute, onEnd, isFinished, this);
    }

    public Command externalWaitUntilHomed() {
        return Commands.run(() -> {}).until(() -> m_isHomed);
    }
    
    public boolean getIsHomed() {
        return m_isHomed;
    }

    @Override
    public void periodic() {

        setCoast(nte_coast.getBoolean(false));

        log_eleAtHeight.accept(nearSetpoint());
        log_elevatorActualMeters.accept(getPositionMeters().in(Meters));
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

    private static final double kInch = 0.169;

    //all these values here are still not 100% exact (CLIMB_UP and CLIMB_DOWN ARE STILL DUMMY VALUES) and will need tweaking
    public enum EleHeight {
        HOME(0.3),
        L1(5.590325),
        L2(5.653564 + kInch),
        L3(8.451660 + (kInch / 2)),
        L4(12.89),
        CLIMB_UP(2.08 - (kInch * 5)), // this height will move the robot up for climb
        CLIMB_DOWN(2.08 - (kInch * 8)), //this height will ove robot down for climb
        HP(2.08 - kInch - 0.18); //human player station intake height

        public final double rotations;
        

        private EleHeight(double rotations){
            this.rotations = rotations;
        }
    }

    public enum AlgaeHeight {
        L2(6.75097),
        L3(9.529046 - 0.17);

        public final double rotations;

        private AlgaeHeight(double rotations){
            this.rotations = rotations;
        }
    }
}