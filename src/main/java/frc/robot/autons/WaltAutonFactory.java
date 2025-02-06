package frc.robot.autons;

import java.util.ArrayList;

import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autons.TrajsAndLocs.HPReefPair;
import frc.robot.autons.TrajsAndLocs.HPStation;
import frc.robot.autons.TrajsAndLocs.ReefHPPair;
import frc.robot.autons.TrajsAndLocs.ReefLocs;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Swerve;
import frc.util.Elastic;
import frc.robot.subsystems.Elevator.EleHeights;
import frc.robot.autons.TrajsAndLocs.Trajectories;

public class WaltAutonFactory {
    private final AutoFactory m_autonFactory;
    private final SequentialCommandGroup m_fullCmdSequence;
    private final AutoRoutine m_routine;

    private final SwerveRequest.SwerveDriveBrake m_brake = new SwerveRequest.SwerveDriveBrake();

    public WaltAutonFactory(AutoFactory autonFactory, Swerve swerve, Elevator ele) {
        m_autonFactory = autonFactory;
        m_routine = m_autonFactory.newRoutine("auton");
        m_fullCmdSequence = new SequentialCommandGroup();
    }

    class AutonCycle {
        public final ReefLocs reefLoc;
        public final EleHeights height;
        public final HPStation hpStation;
        public final ReefHPPair reefHPPair;
        public final HPReefPair hpReefPair;
        Elastic.Notification reefToHPError = 
                    new Elastic.Notification(
                        Elastic.Notification.NotificationLevel.ERROR, 
                        "Invalid reef to HP AutonCycle", 
                        "UR AUTON SUX AND HAS AN INVALID REEF-TO-HP YOU DUMB funnel cake"
                    );
        Elastic.Notification hpToReefError = 
                        new Elastic.Notification(
                            Elastic.Notification.NotificationLevel.ERROR, 
                            "Invalid HP to reef AutonCycle", 
                            "UR AUTON SUX AND HAS AN INVALID HP-TO-REEF YOU DUMB funnel cake");
    
        public AutonCycle(ReefLocs _reef, EleHeights _height, HPStation _hp) {
            reefLoc = _reef;
            height = _height;
            hpStation = _hp;
            reefHPPair = new ReefHPPair(reefLoc, hpStation);
            hpReefPair = new HPReefPair(hpStation, reefLoc);
        }

        public boolean isLegit() {
            if(!Trajectories.ReefToHPTrajs.containsKey(reefHPPair)) {
                // banks said that i actually shouldnt write code that crashes the entire program, which im lowk sad abt. was looking forward to it.
                Elastic.sendNotification(reefToHPError);
                return false;
            }
    
            if(!Trajectories.HPToReefTrajs.containsKey(hpReefPair)) {
                // rip DEATH AND DESTRUCTION you were such a good concept in my head. fly high.
                Elastic.sendNotification(hpToReefError);
                return false;
            }
            return true;
        }
    }

    // public Command generateAuton(Swerve drivetrain, Superstructure superstructure, ArrayList<AutonCycle> cycles) { // im assuming that autoncycle will include 1st iteration
    //     // iterate AutonCycles and validate all HP->Score and Score->HP pairs
    //     for (AutonCycle autonCycle : cycles) {
    //         if(!autonCycle.isLegit()) {
    //             return Commands.print("that one scene in alice in borderland where karube just sits there contemplatively before he blows up.");
    //         }
    //     }

    //     AutoTrajectory firstScoreTraj = m_routine.trajectory(Trajectories.ReefToHPTrajs.get(cycles.get(0).reefHPPair));
    //     AutoTrajectory firstLoadTraj = m_routine.trajectory(Trajectories.HPToReefTrajs.get(cycles.get(0).hpReefPair));

    //     m_routine.active().onTrue(
    //         Commands.sequence(
    //             firstScoreTraj.resetOdometry(),
    //             firstScoreTraj.cmd()
    //         )
    //     );

    //     firstScoreTraj.atTime("eleUp").onTrue()
    // }
}