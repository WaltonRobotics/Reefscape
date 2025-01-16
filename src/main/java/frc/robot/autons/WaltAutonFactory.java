package frc.robot.autons;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutonFactory {
    private final AutoFactory m_factory;

    public AutonFactory(AutoFactory factory) {
        m_factory = factory;
    }

    /* just drivin in a straight line */
    public AutoRoutine ezTest1() {
        final AutoRoutine routine = m_factory.newRoutine("ezTest1");

        //trajs
        AutoTrajectory ezTest1Traj = routine.trajectory("ezTest1");

        routine.active().onTrue(
            Commands.sequence(
                ezTest1Traj.resetOdometry(),
                ezTest1Traj.cmd()
            )
        );

        return routine;
    }
}
