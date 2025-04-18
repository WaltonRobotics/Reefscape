package frc.robot.autons;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.BooleanSupplier;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autons.TrajsAndLocs.ReefLocs;
import frc.robot.autons.TrajsAndLocs.StartingLocs;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Swerve;

public class SimpleAutons {
    public AutoFactory m_autoFactory;
    public Superstructure m_superstructure;

    public SimpleAutons(AutoFactory autofactory, Superstructure superstructure) {
        m_autoFactory = autofactory;
        m_superstructure = superstructure;
    }

    private static BooleanSupplier nearPoseXY(Swerve swerve, Pose2d dest, double toleranceMeters) {
        return () -> {
            double distance = dest.getTranslation().getDistance(swerve.getState().Pose.getTranslation());
            return distance <= 0.25;
        };
    }
    

    public AutoRoutine moveForward() {
        AutoRoutine routine = m_autoFactory.newRoutine("drive forward");

        AutoTrajectory drive = routine.trajectory(TrajsAndLocs.Trajectories.StartToReefTrajs.get(new Pair<StartingLocs , ReefLocs>(StartingLocs.MID_H, ReefLocs.REEF_H)));
        AutoTrajectory drive2 = routine.trajectory(TrajsAndLocs.Trajectories.StartToReefTrajs.get(new Pair<StartingLocs , ReefLocs>(StartingLocs.MID_H, ReefLocs.REEF_H)));

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
