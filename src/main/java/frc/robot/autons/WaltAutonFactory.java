// package frc.robot.autons;

// import java.util.ArrayList;

// import com.ctre.phoenix6.swerve.SwerveRequest;

// import choreo.auto.AutoFactory;
// import choreo.auto.AutoRoutine;
// import choreo.auto.AutoTrajectory;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.autons.TrajsAndLocs.HPReefPair;
// import frc.robot.autons.TrajsAndLocs.HPStation;
// import frc.robot.autons.TrajsAndLocs.ReefHPPair;
// import frc.robot.autons.TrajsAndLocs.ReefLocs;
// import frc.robot.subsystems.Superstructure;
// import frc.robot.subsystems.Swerve;
// import frc.util.Elastic;
// import frc.robot.subsystems.Elevator.EleHeight;
// import frc.robot.autons.TrajsAndLocs.Trajectories;

// public class WaltAutonFactory {
//     private final AutoFactory m_autonFactory;
//     private final AutoRoutine m_routine;

//     private final SwerveRequest.SwerveDriveBrake m_brake = new SwerveRequest.SwerveDriveBrake();

//     public WaltAutonFactory(AutoFactory autonFactory) {
//         m_autonFactory = autonFactory;
//         m_routine = m_autonFactory.newRoutine("auton");
//     }

//     class AutonCycle {
//         public final ReefLocs reefLoc;
//         public final EleHeight height;
//         public final HPStation hpStation;
//         public final ReefHPPair reefHPPair;
//         public final HPReefPair hpReefPair;
//         Elastic.Notification reefToHPError = 
//                     new Elastic.Notification(
//                         Elastic.Notification.NotificationLevel.ERROR, 
//                         "Invalid reef to HP AutonCycle", 
//                         "INVALID REEF-TO-HP: if this auton was a cat, you just killed it."
//                     );
//         Elastic.Notification hpToReefError = 
//                         new Elastic.Notification(
//                             Elastic.Notification.NotificationLevel.ERROR, 
//                             "Invalid HP to reef AutonCycle", 
//                             "INVALID HP-TO-REEF: if this auton was a cat, you just killed it.");
    
//         public AutonCycle(ReefLocs _reef, EleHeight _height, HPStation _hp) {
//             reefLoc = _reef;
//             height = _height;
//             hpStation = _hp;
//             reefHPPair = new ReefHPPair(reefLoc, hpStation);
//             hpReefPair = new HPReefPair(hpStation, reefLoc);
//         }

//         public boolean isLegit() {
//             if(!Trajectories.ReefToHPTrajs.containsKey(reefHPPair)) {
//                 // banks said that i actually shouldnt write code that crashes the entire program, which im lowk sad abt. was looking forward to it.
//                 Elastic.sendNotification(reefToHPError);
//                 return false;
//             }
    
//             if(!Trajectories.HPToReefTrajs.containsKey(hpReefPair)) {
//                 // rip DEATH AND DESTRUCTION you were such a good concept in my head. fly high.
//                 Elastic.sendNotification(hpToReefError);
//                 return false;
//             }
//             return true;
//         }
//     }

//     /*
//      * cycles start from the first time u get to an hp
//      */
//     public Command generateAuton(
//         Swerve drivetrain, 
//         Superstructure superstructure, 
//         AutonChooser firstStartToReef, 
//         AutonChooser firstReefToHP, 
//         EleHeight firstHeight, 
//         ArrayList<AutonCycle> cycles
//     ) { // the autonchooser schtuffs are dummy vars until sohan and xandra figure it out
//         // iterate AutonCycles and validate all HP->Score and Score->HP pairs
//         for (AutonCycle autonCycle : cycles) {
//             if(!autonCycle.isLegit()) {
//                 return Commands.print("that one scene in alice in borderland where karube just sits there contemplatively before he blows up.");
//             }
//         }

//         AutoTrajectory firstScoreTraj = m_routine.trajectory("start to reef");
//         AutoTrajectory firstLoadTraj = m_routine.trajectory("reef to hp");

//         m_routine.active().onTrue(
//             Commands.sequence(
//                 firstScoreTraj.resetOdometry(),
//                 firstScoreTraj.cmd()
//             )
//         );

//         firstScoreTraj.atTime("eleUp")
//             .onTrue(
//                 Commands.sequence(
//                     Commands.runOnce(() -> superstructure.requestIsPreload(true)),
//                     superstructure.requestEleHeight(() -> firstHeight, true),
//                     Commands.runOnce(() -> superstructure.requestIsPreload(false))
//                 )
//             );
//         firstScoreTraj.done().onTrue(
//             Commands.sequence(
//                 Commands.parallel(
//                     drivetrain.applyRequest(() -> m_brake),
//                     superstructure.autonScoreReq()
//                 ),
//                 firstLoadTraj.cmd()
//             )
//         );
//         firstLoadTraj.atTime("intake")
//             .onTrue(superstructure.autonRequestToIntake());
//         // now ur at the HP

//         // list of traj start cmds
//         ArrayList<AutoTrajectory> trajList = new ArrayList<>();
//         for (int cycleIdx = 0; cycleIdx < cycles.size(); cycleIdx++) {
//             // remember, ur at HP rn
//             AutonCycle cycle = cycles.get(cycleIdx);

//             var hpToReefTraj = m_routine.trajectory(Trajectories.HPToReefTrajs.get(cycle.hpReefPair));
//             var reefToHpTraj = m_routine.trajectory(Trajectories.ReefToHPTrajs.get(cycle.reefHPPair));

//             trajList.add(hpToReefTraj);
//             // attach this cycle's first traj to the end of the last cycle's last traj
//             if(cycles.size() > 1 && cycleIdx > 0) {
//                 var lastCycleDone = trajList.get(cycleIdx - 1).done();
                
//                 lastCycleDone.and(superstructure.stateTrg_intook).onTrue(hpToReefTraj.cmd());
//             }

//             hpToReefTraj.atTime("eleUp").onTrue(superstructure.requestEleHeight(() -> cycle.height, true));
            
//             hpToReefTraj.done().onTrue(
//                 Commands.sequence(
//                     Commands.parallel(
//                         drivetrain.applyRequest(() -> m_brake),
//                         superstructure.autonScoreReq()
//                     ),
//                     reefToHpTraj.cmd()
//                 )
//             );

//             reefToHpTraj.atTime("intake")
//                 .onTrue(superstructure.autonRequestToIntake());
//         }

//         firstLoadTraj.done().onTrue(trajList.get(0).cmd());

//         return m_routine.cmd();
//     }
// }