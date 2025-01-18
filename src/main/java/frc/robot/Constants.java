package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

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
        public static final double kMaximumHeight = 10; //meters
        public static final double kStartingHeightMeters = 0; //meters
        public static final double kSensorToMechanismRatio = (kGearRatio) / (Units.inchesToMeters(kSpoolDiameter) * Math.PI);

        public static final double kMeterstoRotations = 1;

        private static final CurrentLimitsConfigs kLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(100)
            .withSupplyCurrentLimit(50)
            .withStatorCurrentLimitEnable(true);
        private static final FeedbackConfigs kFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio((kGearRatio) / (Units.inchesToMeters(kSpoolDiameter) * Math.PI));
        private static final MotionMagicConfigs kMagicConfigs = new MotionMagicConfigs();

        
        public static final TalonFXConfiguration kRightTalonFXConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(kLimitConfigs)
            .withFeedback(kFeedbackConfigs)
            .withMotionMagic(kMagicConfigs);
        public static final TalonFXConfiguration kLeftTalonFXConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(kLimitConfigs)
            .withFeedback(kFeedbackConfigs)
            .withMotionMagic(kMagicConfigs);
    }
}
