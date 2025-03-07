package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.ExternalFeedbackConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;

import edu.wpi.first.units.measure.Current;

import com.ctre.phoenix6.signals.BrushedMotorWiringValue;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorPhaseValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;

//numbers are dummies
public class Constants {
    /* general */
    public static final  boolean kDebugLoggingEnabled = true;
 
    public static final double kRumbleIntensity = 1.0;
    public static final double kRumbleTimeoutSecs = 0.5;
    public static class AlgaeK {
        public static final String kLogTab = "AlgaeSubsys";
        
        public static final int kWristCANID = 12;
        public static final int kIntakeCANID = 13;

        // motor configuration section
        // wrist motor
        public static final int kWristGearRatio = 40;   //TODO: check if still accurate
        public static final int kWristSensorToMechanismRatio = kWristGearRatio;
        public static final int kAngleTolerance = 3; //DUMMY VALUE

        public static final double kWristMMVelo = 1;
        public static final double kWristMMAccel = 25;
        public static final double kWristMMJerk = 300;
        public static final double kWristMMVeloSlow = 0.65;
        public static final double kWristMMAccelSlow = 10;

        private static final CurrentLimitsConfigs kWristCurrentLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(20)
            .withSupplyCurrentLimit(20)
            .withStatorCurrentLimitEnable(true);
        private static final FeedbackConfigs kWristFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(kWristSensorToMechanismRatio);
        private static final MotionMagicConfigs kWristMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(kWristMMVelo))
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(kWristMMAccel))
            .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(kWristMMJerk));
        private static final Slot0Configs kWristSlot0Configs = new Slot0Configs()
            .withKS(0.25)
            .withKV(4)
            .withKA(0)
            .withKP(10) // gg ez
            .withKI(0)
            .withKD(0);
        public static final MotorOutputConfigs kWristMotorOutputConfigs = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake);
        public static final TalonFXConfiguration kWristConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(kWristCurrentLimitConfigs)
            .withFeedback(kWristFeedbackConfigs)
            .withMotionMagic(kWristMagicConfigs)
            .withSlot0(kWristSlot0Configs)
            .withMotorOutput(kWristMotorOutputConfigs);
        
        // intake motor
        public static final int kIntakeGearRatio = 2;
        public static final int kIntakeSensorToMechanismRatio = kIntakeGearRatio;
        public static final double kHasAlgaeCurrent = 10; //DUMMY VALUE
        private static final CurrentLimitsConfigs kIntakeCurrentLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(100)
            .withSupplyCurrentLimit(50)
            .withStatorCurrentLimitEnable(true);
        private static final FeedbackConfigs kIntakeFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(kIntakeSensorToMechanismRatio);
        private static final MotionMagicConfigs kIntakeMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(1))
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10))
            .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));
        private static final Slot0Configs kIntakeSlot0Configs = new Slot0Configs()
            .withKS(0.25)
            .withKV(0.12)
            .withKA(0.01)
            .withKP(60)
            .withKI(0)
            .withKD(0.5);
        public static final MotorOutputConfigs kIntakeMotorOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive);
        public static final TalonFXConfiguration kIntakeConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(kIntakeCurrentLimitConfigs)
            .withFeedback(kIntakeFeedbackConfigs)
            .withMotionMagic(kIntakeMagicConfigs)
            .withSlot0(kIntakeSlot0Configs)
            .withMotorOutput(kIntakeMotorOutputConfigs);

        // state machine work -- make real values!
        public static final Current kIntakeCurrentSpikeThreshold = Amps.of(20);
        public static final Current kShootCurrentDropThreshold = Amps.of(5);
        public static final double kMaxAngleDeg = 120; //TODO: check if this is still accurate - from zero position to algae pickup
    }

    public class Coralk {
        // coral things
        public static final String kLogTab = "CoralSubsys";
        public static final int kCoralMotorCANID = 30; 
        public static final int kTopBeamBreakChannel = 0;
        public static final int kBotBeamBreakChannel = 1;

        // TODO: ask alexandra and hrehaan what these are for (???)
        // public static final double kGearRatio = 1; //for arm spinup and coral intake
        // public static final double kArmGearRatio = 2; //for arm pivot

        public static final double kCoralSpeed = 1;

        public static final TalonFXConfiguration kCoralMotorTalonFXConfiguration = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    }

    public final class FingerK {
        // finger things
        public static final String kLogTab = "FingerSubsys";

        public static final int kFingerMotorCANID = 31;

        public static final double kMaxAngleRotations = 0;
        public static final double kMinAngleRotations = -1;
        public static final double kParallelToGroundRotations = -0.7;
        public static final double kDefaultPos = -0.05;   

        private static final MotorOutputConfigs kMotorOutputConfig = new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);

        private static final CurrentLimitsConfigs kCurrentLimitConfig = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(5)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(3)
            .withSupplyCurrentLimitEnable(true);        
        private static final ClosedLoopGeneralConfigs kClosedLoopGeneralConfig = new ClosedLoopGeneralConfigs()
            .withContinuousWrap(false);

        private static final Slot0Configs kSlot0Config = new Slot0Configs()
            .withKP(100)
            .withKS(1)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        private static final ExternalFeedbackConfigs kExternalFeedbackConfig = new ExternalFeedbackConfigs()
            .withExternalFeedbackSensorSource(ExternalFeedbackSensorSourceValue.Quadrature)
            .withQuadratureEdgesPerRotation(4096)
            .withRotorToSensorRatio(1)
            .withSensorPhase(SensorPhaseValue.Opposed)
            .withSensorToMechanismRatio(2);        
        public static final SoftwareLimitSwitchConfigs kSoftLimitEnabledConfig = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(kMaxAngleRotations)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(kMinAngleRotations);

        public static final SoftwareLimitSwitchConfigs kSoftLimitSwitchDisabledConfig = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(false)
            .withReverseSoftLimitEnable(false);

        private static final CommutationConfigs kCommutationConfig = new CommutationConfigs()
            .withMotorArrangement(MotorArrangementValue.Brushed_DC)
            .withBrushedMotorWiring(BrushedMotorWiringValue.Leads_A_and_B);
        

        public static final TalonFXSConfiguration kTalonFXSConfig = new TalonFXSConfiguration()
            .withMotorOutput(kMotorOutputConfig)
            .withCurrentLimits(kCurrentLimitConfig)
            .withClosedLoopGeneral(kClosedLoopGeneralConfig)
            .withSlot0(kSlot0Config)
            .withExternalFeedback(kExternalFeedbackConfig)
            .withSoftwareLimitSwitch(kSoftLimitEnabledConfig)
            .withCommutation(kCommutationConfig);
    }

    public final class ElevatorK {
        public static final String kLogTab = "EleSubsys";

        public static final int kFrontCANID = 10;
        public static final int kBackCANID = 11;

        public static final double kGearRatio = 50/12;
        public static final Distance kSpoolRadius = Inches.of(0.9175);  // TODO: ask banks if the thing we considered a spool is a spool?

        public static final double kP = 40;
        public static final double kI = 50;
        public static final double kS = 0.5;
        public static final double kV = 0;
        public static final double kA = 0;
        public static final double kG = 0; 

        public static final Mass kCarriageMassKg = Pounds.of(5);
        public static final Distance kMinimumHeight = Feet.of(0);
        public static final Distance kMaximumHeight = Meters.of(15);
        public static final Distance kStartingHeightMeters = Feet.of(0);
        public static final double kTolerancePulleyRotations = 0.1;
        //SensorToMechanismRatio = kGearRatio

        public static LinearVelocity rotationsToMetersVel(AngularVelocity rotations){
            return kSpoolRadius.per(Second).times(rotations.in(RadiansPerSecond));
        }

        public static Angle metersToRotation(Distance meters){
        return Radians.of(meters.in(Meters) / (2 * Math.PI * kSpoolRadius.in(Meters)));
        }

        public static Distance rotationsToMeters(Angle rotations) {
            return Meters.of(rotations.in(Radians) * 2 * Math.PI * kSpoolRadius.in(Meters));
        }

        public static AngularVelocity metersToRotationVel(LinearVelocity meters){
            return RadiansPerSecond.of(meters.in(MetersPerSecond)/kSpoolRadius.in(Meters));
        }

        public static AngularVelocity metersToRotationVel(double metersPerSecond){
            return metersToRotationVel(LinearVelocity.ofBaseUnits(metersPerSecond, MetersPerSecond));
        }

        private static final CurrentLimitsConfigs kCurrentLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(100)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(75)
            .withSupplyCurrentLimitEnable(true);
        private static final FeedbackConfigs kFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(kGearRatio);
        private static final Slot0Configs kSlot0Configs = new Slot0Configs()
            .withKS(kS) 
            .withKV(kV) 
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withKP(kP)
            .withKI(kI) 
            .withKG(kG);
        private static final MotorOutputConfigs kMotorOutputConfigs = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.CounterClockwise_Positive);
        public static final SoftwareLimitSwitchConfigs kSoftwareLimitConfigs = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(12.895)  // true hard 12.9849854
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(0);
        public static final SoftwareLimitSwitchConfigs kSoftLimitSwitchDisabledConfig = new SoftwareLimitSwitchConfigs();
        private static final MotionMagicConfigs kMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(12)
            .withMotionMagicAcceleration(50)
            .withMotionMagicJerk(200);


        public static final TalonFXConfiguration kFrontTalonFXConfig = new TalonFXConfiguration()
            .withCurrentLimits(kCurrentLimitConfigs)
            .withFeedback(kFeedbackConfigs)
            .withMotionMagic(kMotionMagicConfigs)
            .withSlot0(kSlot0Configs)
            .withSoftwareLimitSwitch(kSoftwareLimitConfigs)
            .withMotorOutput(kMotorOutputConfigs);

        public static final TalonFXConfiguration kRearTalonFXConfig = new TalonFXConfiguration()
            .withCurrentLimits(kCurrentLimitConfigs)
            .withMotorOutput(kMotorOutputConfigs);

    }

    public class RobotK {
        public static final String kLogTab = "SuperStructure";
    }
}