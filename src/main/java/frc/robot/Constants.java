package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;

//numbers are dummies
public class Constants {
    /* general */
    public static boolean kDebugLoggingEnabled = true;

    public class Elevatork{
        public static final int kLeftCANID = 10;
        public static final int kRightCANID = 11;

        public static final double kGearRatio = 27;
        public static final double kSpoolDiameter = 2; //inches

        public static final double kCarriageMassKg = 4;
        public static final double kMaximumHeight = 2; //meters
        public static final double kStartingHeightMeters = 0; //meters
        public static final double kSensorToMechanismRatio = (kGearRatio) / (Units.inchesToMeters(kSpoolDiameter) * Math.PI);

        private static final CurrentLimitsConfigs kLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(100)
            .withSupplyCurrentLimit(50)
            .withStatorCurrentLimitEnable(true);
        private static final FeedbackConfigs kFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio((kGearRatio) / (Units.inchesToMeters(kSpoolDiameter) * Math.PI));
        // all of these dummy values are stolen from a ctre example project
        private static final MotionMagicConfigs kMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(5))
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10))
            .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));
        private static final Slot0Configs kSlot0Configs = new Slot0Configs()
            .withKS(0.25)
            .withKV(0.12)
            .withKA(0.01)
            .withKP(60)
            .withKI(0)
            .withKD(0.5);
        
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
