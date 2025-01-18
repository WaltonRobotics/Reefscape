package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

//numbers are dummies
public class Constants {
    /* general */
    public static boolean kDebugLoggingEnabled = true;

    public class Elevatork{
        public static final int kLeftCANID = 10;
        public static final int kRightCANID = 11;

        public static final double kGearRatio = 0.2;
        public static final double kSpoolDiameter = 5.5;

        private static final CurrentLimitsConfigs kLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(100)
            .withSupplyCurrentLimit(50)
            .withStatorCurrentLimitEnable(true);
        private static final FeedbackConfigs kFeedbackConfigs = new FeedbackConfigs();
        private static final MotionMagicConfigs kMagicConfigs = new MotionMagicConfigs();

        public static final TalonFXConfiguration kLeftTalonFXConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(kLimitConfigs)
            .withFeedback(kFeedbackConfigs)
            .withMotionMagic(kMagicConfigs);
        public static final TalonFXConfiguration kRightTalonFXConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(kLimitConfigs)
            .withFeedback(kFeedbackConfigs)
            .withMotionMagic(kMagicConfigs);

    }
}
