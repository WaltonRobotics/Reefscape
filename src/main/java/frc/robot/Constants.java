package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;

//numbers are dummies
public class Constants {
    /* general */
    public static boolean kDebugLoggingEnabled = true;

    public class ElevatorK{
        public static final int kLeftCANID = 10;
        public static final int kRightCANID = 11;

        public static final double kGearRatio = 50.0/11.0;
        public static final Distance kSpoolRadius = Inches.of(1);

        public static final double kV = 0;
        public static final double kA = 0;
        public static final double kG = 0; 

        public static final Mass kCarriageMassKg = Pounds.of(5);
        public static final Distance kMinimumHeight = Feet.of(0);
        public static final Distance kMaximumHeight = Feet.of(8);
        public static final Distance kStartingHeightMeters = Feet.of(0);
        //SensorToMechanismRatio = kGearRatio

        public static LinearVelocity rotationsToMetersVel(AngularVelocity rotations){
            return kSpoolRadius.per(Second).times(rotations.in(RadiansPerSecond));
        }

        public static Angle metersToRotation(Distance meters){
            return Radians.of(meters.in(Meters) / kSpoolRadius.in(Meters));
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
}
