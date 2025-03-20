package frc.robot.autons;

import static edu.wpi.first.units.Units.Meters;

import java.util.Set;
import java.util.function.BooleanSupplier;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.autons.TrajsAndLocs.ReefLocs;
import frc.robot.autons.TrajsAndLocs.StartingLocs;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Swerve.*;

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
    

    // move these maybe
    private static final Transform2d partnerPushBlue = new Transform2d(Meters.of(0.4), Meters.of(0), Rotation2d.kZero);
    private static final Transform2d partnerPushRed = new Transform2d(Meters.of(-0.4), Meters.of(0), Rotation2d.kZero);

    private static Command pushPartnerNeedsDefer(Swerve swerve) {
        Pose2d startPose = swerve.getState().Pose;
        Transform2d transform = partnerPushBlue;
        var allianceOpt = DriverStation.getAlliance();
        if (allianceOpt.isPresent() && allianceOpt.get().equals(Alliance.Red)) {
            transform = partnerPushRed;
        }
        Pose2d destinationPose = startPose.transformBy(transform);

        System.out.println("PUSH_PARTNER Going from " + startPose.toString() + " to " + destinationPose.toString());

        return Commands.sequence(
            swerve.moveToPose(destinationPose),
            Commands.print("Moving to dest"),
            Commands.waitUntil(nearPoseXY(swerve, destinationPose, 0.05)),
            Commands.print("Returning to start"),
            swerve.moveToPose(startPose),
            Commands.waitUntil(nearPoseXY(swerve, startPose, 0.05))
        );
    }

    public static Command pushPartner(Swerve swerve) {
        return Commands.defer(() -> pushPartnerNeedsDefer(swerve), Set.of());
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
