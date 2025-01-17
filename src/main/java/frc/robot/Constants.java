package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class Constants {
    /* general */
    public static boolean kDebugLoggingEnabled = true;

    public class EleK {
        public static final TalonFXConfiguration kLeftConfigs = new TalonFXConfiguration();
        public static final TalonFXConfiguration kRightConfigs = new TalonFXConfiguration();

        public static final CANcoderConfiguration kCANcoderConfigs = new CANcoderConfiguration();

        // ermmmmmmm
        static {
            kLeftConfigs.CurrentLimits = kLeftConfigs.CurrentLimits
                .withSupplyCurrentLimit(1)
                .withSupplyCurrentLimitEnable(true);
            kRightConfigs.CurrentLimits = kRightConfigs.CurrentLimits
                .withSupplyCurrentLimit(1)
                .withSupplyCurrentLimitEnable(true);

            kLeftConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1;
            kLeftConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            kLeftConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 1;
            kLeftConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

            kRightConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1;
            kRightConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            kRightConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 1;
            kRightConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
           
            kLeftConfigs.Slot0.kP = 1;
            kLeftConfigs.Slot0.kD = 1;

            kRightConfigs.Slot0.kP = 1;
            kRightConfigs.Slot0.kD = 1;

            // TODO: add cancoder configs thingies maybe?
        }
    }
}
