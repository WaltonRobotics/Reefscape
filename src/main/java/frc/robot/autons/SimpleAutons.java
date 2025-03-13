package frc.robot.autons;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.autons.TrajsAndLocs.ReefLocs;
import frc.robot.autons.TrajsAndLocs.StartingLocs;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Swerve.*;

public class SimpleAutons {
    public AutoFactory m_autoFactory;
    public Superstructure m_superstructure;

    public SimpleAutons(AutoFactory autofactory, Superstructure superstructure) {
        m_autoFactory = autofactory;
        m_superstructure = superstructure;
    }

    public AutoRoutine moveForward() {
        AutoRoutine routine = m_autoFactory.newRoutine("drive forward");

        AutoTrajectory drive = routine.trajectory(TrajsAndLocs.Trajectories.StartToReefTrajs.get(new Pair<StartingLocs , ReefLocs>(StartingLocs.MID, ReefLocs.REEF_H)));
        AutoTrajectory drive2 = routine.trajectory(TrajsAndLocs.Trajectories.StartToReefTrajs.get(new Pair<StartingLocs , ReefLocs>(StartingLocs.MID, ReefLocs.REEF_H)));

        routine.active().onTrue(
            Commands.sequence(
                drive.resetOdometry(),
                drive.cmd()
            )
        );

        drive.done()
            .onTrue(
                Commands.sequence(
                    m_superstructure.autonEleToHPReq(),
                    Commands.waitSeconds(2),
                    drive2.cmd()
                )
            );

        return routine;
    }
}
