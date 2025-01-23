package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.util.Units;

public class Constants {
    /* general */
    public static boolean kDebugLoggingEnabled = true;

    public class Coralk {
        public static final int kCoralMotorCANID = 1; //TODO: check real CANID

        public static final double kGearRatio = 27; //TODO: check real gear ratio
        public static final double kSpoolDiameter = 2; //TODO: check real spool diameter

        private static final CurrentLimitsConfigs kLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(100)    
            .withSupplyCurrentLimit(50)
            .withStatorCurrentLimitEnable(true);
            //TODO: ^check what the values actually should be^

        private static final FeedbackConfigs kFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio((kGearRatio) / (Units.inchesToMeters(kSpoolDiameter) * Math.PI));

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

        public static final TalonFXConfiguration kCoralMotorTalonFXConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(kLimitConfigs)
            .withFeedback(kFeedbackConfigs)
            .withMotionMagic(kMagicConfigs)
            .withSlot0(kSlot0Configs);
    }
    
}
