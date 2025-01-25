package frc.robot.autons;

import java.util.ArrayList;

import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
    final AutoRoutine m_routine;
    private ArrayList<AutoTrajectory> m_trajList = new ArrayList<AutoTrajectory>();
    private double eleWaitSecs = 1; //dummy num

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    public WaltAutonFactory(AutoFactory autoFactory, Swerve swerve, Elevator ele) {
        m_trajs.configureTrajectories();
        m_autoFactory = autoFactory;
        m_routine = m_autoFactory.newRoutine("auton");
        m_cmdSched = new SequentialCommandGroup(/* TODO: resetOdometry */);
    }

    public SequentialCommandGroup generateAuton(Swerve drivetrain, Elevator ele, /* TODO: coral */ FirstScoringLocs firstScoreLoc, ArrayList<ScoringLocs> scoreLocs, ArrayList<EleHeights> eleHeights, ArrayList<CS> CSLocs) {
        ScoringLocs firstLoc = ScoringLocs.getSameLoc(firstScoreLoc);

        /* iteration 1 */
        m_cmdSched.addCommands(
            /* scoring the first piece */
            Commands.parallel(
                m_routine.trajectory(firstScoreLoc.m_startAndTraj.getSecond()).cmd(),
                Commands.sequence(
                    ele.setPosition(EleHeights.HOME).asProxy(), //ele cmd: check later
                    Commands.waitSeconds(eleWaitSecs), //dummy num
                    ele.setPosition(eleHeights.get(0)).asProxy()
                )
            ),
            drivetrain.applyRequest(() -> brake).asProxy(),
            /* coral score */

            /* going to cs */
            Commands.parallel(
                m_routine.trajectory(m_trajs.m_toCSTrajMap.get(new Pair<ScoringLocs, CS>(firstLoc, CSLocs.get(0)))).cmd(),
                Commands.sequence(
                    ele.setPosition(EleHeights.HOME).asProxy(),
                    Commands.waitSeconds(eleWaitSecs),
                    ele.setPosition(EleHeights.CS).asProxy()
                )
            ),
            drivetrain.applyRequest(() -> brake).asProxy()
            /* coral intake */
        );

        /* rest of the iterations */
        for (int i = 0; i < scoreLocs.size(); i++) {
            m_trajList.add(m_routine.trajectory(m_trajs.m_toRTrajMap.get(new Pair<CS, ScoringLocs>(CSLocs.get(i), scoreLocs.get(i)))));

            if(CSLocs.size() >= i + 1) {
                m_trajList.add(m_routine.trajectory(m_trajs.m_toCSTrajMap.get(new Pair<ScoringLocs, CS>(scoreLocs.get(i), CSLocs.get(i + 1)))));
            }
        }

        int heightCounter = 1;
        for (int i = 0; i < m_trajList.size(); i++) {
            m_cmdSched.addCommands(
                Commands.parallel(
                    m_trajList.get(i).cmd(),
                    Commands.sequence(
                        ele.setPosition(EleHeights.HOME).asProxy(),
                        Commands.waitSeconds(eleWaitSecs),
                        ele.setPosition(eleHeights.get(heightCounter)).asProxy()
                    )
                ),
                drivetrain.applyRequest(() -> brake).asProxy()
                /* coral score */
            );

            heightCounter++;

            if(m_trajList.size() >= i + 1) {
                i++;
                m_cmdSched.addCommands(
                    Commands.parallel(
                        m_trajList.get(i).cmd(),
                        Commands.sequence(
                            ele.setPosition(EleHeights.HOME).asProxy(),
                            Commands.waitSeconds(eleWaitSecs),
                            ele.setPosition(EleHeights.CS).asProxy()
                        )
                    ),
                    drivetrain.applyRequest(() -> brake).asProxy()
                    /* coral intake */
                );
            }
        }

        return m_cmdSched;
    }

    public AutoRoutine getAuton() {
        m_routine.active().onTrue(m_cmdSched);

        return m_routine;
    }
}
