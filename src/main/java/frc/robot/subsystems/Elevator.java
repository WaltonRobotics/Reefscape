package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Elevatork;
import frc.robot.generated.TunerConstants;

//numbers are dummies
public class Elevator{
    private final TalonFX m_left = new TalonFX(Elevatork.kLeftCANID, TunerConstants.kCANBus);
	private final TalonFX m_right = new TalonFX(Elevatork.kRightCANID, TunerConstants.kCANBus);
    private final Follower m_follower = new Follower(m_right.getDeviceID(),true);
    private final MotionMagicExpoVoltage m_MMEVRequest = new MotionMagicExpoVoltage(0);

    public Elevator(){
        m_left.setControl(m_follower);
        m_left.getConfigurator();
        m_right.getConfigurator();

    }

    public Command setPosition(double heightMeters){
        return Commands.runOnce(() -> m_right.setControl(m_MMEVRequest.withPosition(heightMeters)));
    }

    public Command setPosition(HeightPosition heightMeters){
        return setPosition(heightMeters.m_heightMeters);
    }

    public enum HeightPosition{
        HOME(2),
        L1(5),
        L2(6),
        L3(8),
        L4(10);

        public final double m_heightMeters;

        private HeightPosition(double heightMeters){
            m_heightMeters = heightMeters;
        }

    }

}