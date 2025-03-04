package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.units.measure.Current;

import edu.wpi.first.units.measure.Current;

import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.signals.GravityTypeValue;

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

    // TODO: NONE OF THESE ARE REAL NUMBERS!!!!!!!!!!!!!!!!!
    // BE WARY OF MOTOR NUMBERS - MANY ARE STOLEN FROM ELEVATOR!!!!! (couldn't easily find better ones)
    public static class AlgaeK {
        public static final String kLogTab = "AlgaeSubsys";
        
        public static final int kWristCANID = 12;
        public static final int kIntakeCANID = 13;

        // motor configuration section
        // wrist motor
        public static final int kWristGearRatio = 40;   //TODO: check if still accurate
        public static final int kWristSensorToMechanismRatio = kWristGearRatio;
        public static final int kAngleTolerance = 3;//DUMMY VALUE
        private static final CurrentLimitsConfigs kWristCurrentLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(100)
            .withSupplyCurrentLimit(50)
            .withStatorCurrentLimitEnable(true);
        private static final FeedbackConfigs kWristFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(kWristSensorToMechanismRatio);
        private static final MotionMagicConfigs kWristMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(1))
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10))
            .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));
        private static final Slot0Configs kWristSlot0Configs = new Slot0Configs()
            .withKS(0.25)
            .withKV(0.12)
            .withKA(0.01)
            .withKP(60)
            .withKI(0)
            .withKD(0.5);
        public static final TalonFXConfiguration kWristConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(kWristCurrentLimitConfigs)
            .withFeedback(kWristFeedbackConfigs)
            .withMotionMagic(kWristMagicConfigs)
            .withSlot0(kWristSlot0Configs);

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
        public static final TalonFXConfiguration kIntakeConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(kIntakeCurrentLimitConfigs)
            .withFeedback(kIntakeFeedbackConfigs)
            .withMotionMagic(kIntakeMagicConfigs)
            .withSlot0(kIntakeSlot0Configs);

        // state machine work -- make real values!
        public static final Current kIntakeCurrentSpikeThreshold = Amps.of(20);
        public static final Current kShootCurrentDropThreshold = Amps.of(5);
        public static final double kMaxAngleDeg = 120; //TODO: check if this is still accurate - from zero position to algae pickup
    }

    public class Coralk {
        public static final String kLogTab = "EleSubsys";
        public static final int kCoralMotorCANID = 30; 
        public static final int kFingerMotorCANID = 31;
        public static final int kTopBeamBreakChannel = 3;
        public static final int kBotBeamBreakChannel = 4;

        public static final double kGearRatio = 1; //for arm spinup and coral intake
        public static final double kArmGearRatio = 2; //for arm pivot
        // public static final double kSpoolDiameter = 1; no actual spool diameter

        public static final double kCoralSpeed = 1; //TODO: make frsies

        private static final CurrentLimitsConfigs kLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(100)    
            .withSupplyCurrentLimit(50)
            .withStatorCurrentLimitEnable(true);
            //TODO: ^check what the values actually should be^

        private static final FeedbackConfigs kFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio((kGearRatio));

        // private static final FeedbackConfigs kFeedbackConfigs = new FeedbackConfigs()
        //     .withSensorToMechanismRatio((kGearRatio) / (Units.inchesToMeters(kSpoolDiameter) * Math.PI));

        private static final MotionMagicConfigs kMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(5))
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10))
            .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));
            //TODO: ^check what the values actually should be^
        
        private static final Slot0Configs kSlot0Configs = new Slot0Configs()
            .withKS(0.25)
            .withKV(0.12)
            .withKA(0.01)
            .withKP(60)
            .withKI(0)
            .withKD(0.5);
            //TODO: ^check what the values actually should be^

        public static final TalonFXConfiguration kCoralMotorTalonFXConfiguration = new TalonFXConfiguration();
    }

    public class ElevatorK{
        public static final String kLogTab = "EleSubsys";

        public static final int kFrontCANID = 10;
        public static final int kBackCANID = 11;

        public static final double kGearRatio = 50/12;
        public static final Distance kSpoolRadius = Inches.of(0.9175);  // TODO: ask banks if the thing we considered a spool is a spool?

        public static final double kV = 0;
        public static final double kA = 0;
        public static final double kG = 0; 

        public static final Mass kCarriageMassKg = Pounds.of(5);
        public static final Distance kMinimumHeight = Feet.of(0);
        public static final Distance kMaximumHeight = Meters.of(8);
        public static final Distance kStartingHeightMeters = Feet.of(0);
        public static final double kTolerancePulleyRotations = metersToRotation(Meters.of(0.01)).in(Rotations);
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

        private static final CurrentLimitsConfigs kLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(300)
            .withStatorCurrentLimitEnable(false)
            .withSupplyCurrentLimit(75)
            .withSupplyCurrentLimitEnable(false);
        private static final FeedbackConfigs kFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(kGearRatio);

        private static final MotionMagicConfigs kMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(5)
            .withMotionMagicAcceleration(15)
            .withMotionMagicExpo_kV(3.54);
        private static final Slot0Configs kSlot0Configs = new Slot0Configs()
            .withKS(0.25) 
            .withKV(kV) 
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withKP(10) 
            .withKG(kG);
        
        public static final TalonFXConfiguration kRightTalonFXConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(kLimitConfigs)
            .withFeedback(kFeedbackConfigs)
            .withMotionMagic(kMagicConfigs)
            .withSlot0(kSlot0Configs);

        public static final TalonFXConfiguration kLeftTalonFXConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(kLimitConfigs)
            .withFeedback(kFeedbackConfigs)
            .withMotionMagic(kMagicConfigs)
            .withSlot0(kSlot0Configs);

    }

    public class RobotK {
        public static final String kLogTab = "SuperStructure";
    }
}