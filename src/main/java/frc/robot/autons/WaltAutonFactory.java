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
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Elevator.EleHeights;
import frc.robot.subsystems.Superstructure.CoralState;

public class WaltAutonFactory {
    private final AutoFactory m_autoFactory;
    private TrajsAndLocs.Trajectories m_trajs = new TrajsAndLocs.Trajectories();
    private SequentialCommandGroup m_cmdSched;
    final AutoRoutine m_routine;
    private ArrayList<AutoTrajectory> m_trajList = new ArrayList<AutoTrajectory>();
    private double m_eleWaitSecs = 1; //dummy num

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    public WaltAutonFactory(AutoFactory autoFactory, Swerve swerve, Elevator ele) {
        m_trajs.configureTrajectories();
        m_autoFactory = autoFactory;
        m_routine = m_autoFactory.newRoutine("auton");
        m_cmdSched = new SequentialCommandGroup();
    }

    public SequentialCommandGroup generateAuton(Superstructure superstructure, Swerve drivetrain, Elevator ele, Coral coral, FirstScoringLocs firstScoreLoc, ArrayList<ScoringLocs> scoreLocs, ArrayList<EleHeights> eleHeights, ArrayList<CS> CSLocs) {
        ScoringLocs firstLoc = ScoringLocs.getSameLoc(firstScoreLoc);
        AutoTrajectory firstRTraj = m_routine.trajectory(firstScoreLoc.m_startAndTraj.getSecond());
        AutoTrajectory firstCSTraj = m_routine.trajectory(m_trajs.m_toCSTrajMap.get(new Pair<ScoringLocs, CS>(firstLoc, CSLocs.get(0))));

        /* iteration 1 */
        m_cmdSched.addCommands(
            /* 
             * scoring the first piece 
            */
            firstRTraj.resetOdometry(),
            Commands.runOnce(() -> superstructure.autonPreload()), //SM -> INTOOK
            Commands.parallel(
                firstRTraj.cmd(),
                Commands.sequence(
                    Commands.waitSeconds(m_eleWaitSecs), //dummy num
                    ele.setPosition(eleHeights.get(0)).asProxy() // once it reaches pos, SM -> SCORE_READY
                    )
                ),
            drivetrain.applyRequest(() -> brake).asProxy(),
            Commands.parallel(
                Commands.runOnce(() -> superstructure.autonScore()), // SM -> SCORING
                Commands.waitUntil(() -> superstructure.getCoralState().equals(CoralState.IDLE))
            ),

            /* going to cs */
            firstCSTraj.resetOdometry(),
            Commands.parallel(
                firstCSTraj.cmd(),
                Commands.parallel(
                    ele.toHome().asProxy(),
                    Commands.sequence(
                        Commands.waitSeconds(m_eleWaitSecs),
                        ele.toCS().asProxy()
                    )
                )
            ),
            drivetrain.applyRequest(() -> brake).asProxy(),
            Commands.parallel(
                Commands.runOnce(() -> superstructure.autonScore()),
                Commands.waitUntil(() -> superstructure.getCoralState().equals(CoralState.INTOOK))
            )
        );

        /* rest of the iterations */
        for (int i = 0; i < scoreLocs.size(); i++) {
            m_trajList.add(m_routine.trajectory(m_trajs.m_toRTrajMap.get(new Pair<CS, ScoringLocs>(CSLocs.get(i), scoreLocs.get(i)))));

            if(CSLocs.size() >= i + 2) { // thats right, right? am i tripping; i might be tripping (im rlly smart, i know i know)
                m_trajList.add(m_routine.trajectory(m_trajs.m_toCSTrajMap.get(new Pair<ScoringLocs, CS>(scoreLocs.get(i), CSLocs.get(i + 1)))));
            }
        }

        int heightCounter = 1;
        for (int i = 0; i < m_trajList.size(); i++) {
            m_cmdSched.addCommands(
                m_trajList.get(i).resetOdometry(), // im actually a tad unsure if ur supposed to reset odometry before every path or j the first one. will find out at some pt  
                Commands.parallel(
                    m_trajList.get(i).cmd(),
                    Commands.parallel(
                        ele.toHome().asProxy(),
                        Commands.sequence(
                            Commands.waitSeconds(m_eleWaitSecs),
                            ele.setPosition(eleHeights.get(heightCounter)).asProxy()
                        )
                    )
                ),
                drivetrain.applyRequest(() -> brake).asProxy(),
                Commands.parallel(
                    Commands.runOnce(() -> superstructure.autonScore()),
                    Commands.waitUntil(() -> superstructure.getCoralState().equals(CoralState.IDLE))
                )
            );

            heightCounter++;

            if(m_trajList.size() >= i + 2) {
                i++;
                m_cmdSched.addCommands(
                    m_trajList.get(i).resetOdometry(),
                    Commands.parallel(
                        m_trajList.get(i).cmd(),
                        Commands.parallel(
                            ele.toHome().asProxy(),
                            Commands.sequence(
                                Commands.waitSeconds(m_eleWaitSecs),
                                ele.toCS().asProxy()
                            )
                        )
                    ),
                    drivetrain.applyRequest(() -> brake).asProxy(),
                    Commands.waitUntil(() -> superstructure.getCoralState().equals(CoralState.INTOOK))
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
