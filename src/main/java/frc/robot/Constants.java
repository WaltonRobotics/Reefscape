package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

//numbers are dummies
public class Constants {
    /* general */
    public static boolean kDebugLoggingEnabled = true;

    public class ElevatorK{
        public static final int kLeftCANID = 10;
        public static final int kRightCANID = 11;

        public static final double kGearRatio = 27;
        public static final Distance kSpoolRadius = Inches.of(1);

        public static final Mass kCarriageMassKg = Kilograms.of(4);
        public static final Distance kMinimumHeight = Meters.of(0);
        public static final Distance kMaximumHeight = Meters.of(6);
        public static final Distance kStartingHeightMeters = Meters.of(0);
        public static final double kSensorToMechanismRatio = (kGearRatio) / (kSpoolRadius.in(Meters) * Math.PI);

        private static final CurrentLimitsConfigs kLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(100)
            .withSupplyCurrentLimit(50)
            .withStatorCurrentLimitEnable(true);
        private static final FeedbackConfigs kFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio((kGearRatio) / (kSpoolRadius.in(Meters) * Math.PI));
        // all of these dummy values are stolen from a ctre example project
        private static final MotionMagicConfigs kMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(5/10))
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10/10))
            .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100/10));
        private static final Slot0Configs kSlot0Configs = new Slot0Configs()
            .withKS(0.25/10)
            .withKV(0.12/10)
            .withKA(0.01/10)
            .withKP(60/10)
            .withKI(0)
            .withKD(0.5/10);
        
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
