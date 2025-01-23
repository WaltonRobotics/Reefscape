package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class Constants {
    /* general */
    public static boolean kDebugLoggingEnabled = true;

    // TODO: NONE OF THESE ARE REAL NUMBERS!!!!!!!!!!!!!!!!!
    // BE WARY OF MOTOR NUMBERS - MANY ARE STOLEN FROM ELEVATOR!!!!! (couldn't easily find better ones)
    public static class AlgaeK {
        public static final int kWristCANID = 12;
        public static final int kIntakeCANID = 13;

        // motor configuration section
        // wrist motor
        public static final int kWristGearRatio = 10;
        public static final int kWristSensorToMechanismRatio = kWristGearRatio;
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
        public static final int kIntakeGearRatio = 10;
        public static final int kIntakeSensorToMechanismRatio = kIntakeGearRatio;
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
    }
}
