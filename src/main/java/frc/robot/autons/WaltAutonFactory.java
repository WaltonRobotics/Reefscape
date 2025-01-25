package frc.robot.autons;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import javax.tools.JavaFileManager.Location;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.SwerveDriveBrake;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutonFactory;
import frc.robot.autons.TrajsAndLocs.CS;
import frc.robot.autons.TrajsAndLocs.FirstScoringLocs;
import frc.robot.autons.TrajsAndLocs.ScoringLocs;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Elevator.EleHeights;

public class WaltAutonFactory {
    private final AutoFactory m_autoFactory;
    private TrajsAndLocs.Trajectories m_trajs = new TrajsAndLocs.Trajectories();
    private SequentialCommandGroup m_cmdSched;

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    public WaltAutonFactory(AutoFactory autoFactory, Swerve swerve, Elevator ele) {
        m_trajs.configureTrajectories();
        m_autoFactory = autoFactory;
        m_cmdSched = new SequentialCommandGroup(/* TODO: resetOdometry */);
    }

    public AutoRoutine generateAuton(Swerve drivetrain, Elevator ele, /* TODO: coral */ FirstScoringLocs firstScoreLoc, ArrayList<ScoringLocs> scoreLocs, ArrayList<EleHeights> eleHeights, ArrayList<CS> CSLocs) {
        final AutoRoutine routine = m_autoFactory.newRoutine("auton");
        ScoringLocs firstLoc = ScoringLocs.getSameLoc(firstScoreLoc);

        /* iteration 1 */
        m_cmdSched.addCommands(
            /* scoring the first piece */
            Commands.parallel(
                routine.trajectory(firstScoreLoc.m_startAndTraj.getSecond()).cmd(),
                Commands.sequence(
                    ele.setPosition(EleHeights.HOME).asProxy(), //ele cmd: check later
                    Commands.waitSeconds(1), //dummy num
                    ele.setPosition(eleHeights.get(0)).asProxy()
                )
            ),
            drivetrain.applyRequest(() -> brake).asProxy(),
            /* coral score */

            /* going to cs */
            Commands.parallel(
                routine.trajectory(m_trajs.m_trajMap.get(new Pair<ScoringLocs, CS>(firstLoc, CSLocs.get(0)))).cmd(),
                Commands.sequence(
                    ele.setPosition(EleHeights.HOME).asProxy(),
                    Commands.waitSeconds(1),
                    ele.setPosition(eleHeights.)
                )
            )
        );


    }
}
