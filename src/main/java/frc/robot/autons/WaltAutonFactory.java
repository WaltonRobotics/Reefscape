package frc.robot.autons;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;

public class WaltAutonFactory {
    private final AutoFactory m_factory;

    public WaltAutonFactory(AutoFactory factory) {
        m_factory = factory;
    }

    /* just drivin in a straight line */
    public AutoRoutine ezTest1() {
        final AutoRoutine routine = m_factory.newRoutine("ezTest1");

        //trajs
        AutoTrajectory ezTest1Traj = routine.trajectory("ez_test_1");

        routine.active().onTrue(
            Commands.sequence(
                ezTest1Traj.resetOdometry(),
                ezTest1Traj.cmd()
            )
        );

        return routine;
    }

    public AutoRoutine middle_2pc() {
        final AutoRoutine routine = m_factory.newRoutine("middle_2pc");

        //trajs
        AutoTrajectory middle_2pc_1 = routine.trajectory("middle_2pc_1");
        AutoTrajectory middle_2pc_2 = routine.trajectory("middle_2pc_2");
        AutoTrajectory middle_2pc_3 = routine.trajectory("middle_2pc_3");

        routine.active().onTrue(
            Commands.sequence(
                middle_2pc_1.resetOdometry(),
                middle_2pc_1.cmd(),
                middle_2pc_2.resetOdometry(),
                middle_2pc_2.cmd(),
                middle_2pc_3.resetOdometry(),
                middle_2pc_3.cmd()
            )
        );

        return routine;
    }
}
