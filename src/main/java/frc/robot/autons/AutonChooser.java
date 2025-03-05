package frc.robot.autons;

import java.util.ArrayList;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autons.TrajsAndLocs.HPStation;
import frc.robot.autons.TrajsAndLocs.ReefLocs;
import frc.robot.autons.TrajsAndLocs.StartingLocs;
// import frc.robot.autons.WaltAutonFactory.AutonCycle;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Elevator.EleHeight;
import frc.robot.subsystems.Swerve;

public class AutonChooser {
    public static final AutoChooser autoChooser = new AutoChooser();

    public static void addPathsAndCmds(WaltAutonFactory autonFactory) {
        autoChooser.addRoutine("auton", () -> autonFactory.generateAuton());
        autoChooser.addRoutine("leave-only", () -> autonFactory.leaveOnly());
        autoChooser.select("auton");

        SmartDashboard.putData("AutonChooser", autoChooser);
}

//     // auton factory
//     private static final Swerve drivetrain = TunerConstants.createDrivetrain();
//     private static final AutoFactory autoFactory = drivetrain.createAutoFactory();
//     private static final WaltAutonFactory autonFactory = new WaltAutonFactory(autoFactory);

//     // ---- Initial
//     // Define Initial Choosers
//     public static SendableChooser<NumCycles> cyclesChooser = new SendableChooser<NumCycles>();
//     public static SendableChooser<EleHeight> startingHeightChooser = new SendableChooser<EleHeight>();
//     public static SendableChooser<StartingLocs> startingPositionChooser = new SendableChooser<StartingLocs>();
//     public static SendableChooser<ReefLocs> firstScoringChooser = new SendableChooser<ReefLocs>();
//     public static SendableChooser<HPStation> firstToHPStationChooser = new SendableChooser<HPStation>();

//     // default, initial values
//     public static NumCycles m_cycles = NumCycles.CYCLE_1;
//     public static EleHeight startingHeight = EleHeight.L4;
//     public static StartingLocs startingPosition = StartingLocs.MID;
//     public static ReefLocs scoringPosition = ReefLocs.REEF_A;
//     public static HPStation hpStation = HPStation.HP_LEFT;

//     static {
//         SmartDashboard.putData("Number of Cycles", cyclesChooser);
//         SmartDashboard.putData("Starting Position Chooser", startingPositionChooser);   
//         SmartDashboard.putData("Starting Elevator Height Chooser", startingHeightChooser);
//         SmartDashboard.putData("First Scoring Chooser", firstScoringChooser);
//         SmartDashboard.putData("First HP Station", firstToHPStationChooser);
//     }

//     // adds the options
//     public static void configureFirstCycle() {
//         // add cycle options
//         cyclesChooser.addOption("1 Cycle", NumCycles.CYCLE_1);
//         cyclesChooser.addOption("2 Cycle", NumCycles.CYCLE_2);
//         cyclesChooser.addOption("3 Cycle", NumCycles.CYCLE_3);
//         cyclesChooser.addOption("4 Cycle", NumCycles.CYCLE_4);

//         // add Starting Position options
//         startingPositionChooser.addOption("Left", StartingLocs.LEFT);
//         startingPositionChooser.addOption("Middle", StartingLocs.MID);
//         startingPositionChooser.addOption("Right", StartingLocs.RIGHT);

//         // changing the starting position AFFECTS HERE - see robot periodic
//         if (startingPosition.equals(StartingLocs.LEFT)) {
//             for (ReefLocs loc : TrajsAndLocs.ReefLocs.OptimalLeftStartCycles) {
//                 firstScoringChooser.addOption(loc.name(), loc);
//             }
//         } else if (startingPosition.equals(StartingLocs.MID)) {
//             for (ReefLocs loc : TrajsAndLocs.ReefLocs.OptimalMidStartCycles) {
//                 firstScoringChooser.addOption(loc.name(), loc);
//             }
//         } else {
//             for (ReefLocs loc : TrajsAndLocs.ReefLocs.OptimalRightStartCycles) {
//                 firstScoringChooser.addOption(loc.name(), loc);
//             }
//         }

//         // add Starting Height options
//         addEleOptions(startingHeightChooser);

//         // add HP station options
//         addHPOptions(firstToHPStationChooser);
//     }

//     // ---- Cycles
//     // define cycle choosers
//     public static ArrayList<SendableChooser<EleHeight>> eleHeightChoosers = new ArrayList<SendableChooser<EleHeight>>();
//     public static ArrayList<SendableChooser<ReefLocs>> hpToReefChoosers = new ArrayList<SendableChooser<ReefLocs>>();
//     public static ArrayList<SendableChooser<HPStation>> reefToHPChoosers = new ArrayList<SendableChooser<HPStation>>();

//     // TODO: MAYBE add method that closes unnecessary sendable choosers (when numCycles Changes)

//     // adds choosers based on number of cycles selected
//     public static void configureCycles() {
//         for (int i = 0; i < m_cycles.m_cycles; i++) {
//             eleHeightChoosers.add(new SendableChooser<EleHeight>());
//             hpToReefChoosers.add(new SendableChooser<ReefLocs>());
//             reefToHPChoosers.add(new SendableChooser<HPStation>());

//             addEleOptions(eleHeightChoosers.get(i));
//             addReefOptions(hpToReefChoosers.get(i));
//             addHPOptions(reefToHPChoosers.get(i));

//             SmartDashboard.putData("HP To Reef Chooser - Cycle " + (i + 1), hpToReefChoosers.get(i));
//             SmartDashboard.putData("Elevator Height - Cycle " + (i + 1), eleHeightChoosers.get(i));   
//             SmartDashboard.putData("Reef to HP Chooser - Cycle " + (i + 1), reefToHPChoosers.get(i));
//         }
//     }

//     // ---- add Options
//     // add elevator height options
//     private static void addEleOptions(SendableChooser<EleHeight> eleHeight) {
//         eleHeight.addOption("L1", EleHeight.L1);
//         eleHeight.addOption("L2", EleHeight.L2);
//         eleHeight.addOption("L3", EleHeight.L3);
//         eleHeight.addOption("L4", EleHeight.L4);
//     }

//     // add reef options
//     private static void addReefOptions(SendableChooser<ReefLocs> reef) {
//         reef.addOption("A", ReefLocs.REEF_A);
//         reef.addOption("B", ReefLocs.REEF_B);
//         reef.addOption("C", ReefLocs.REEF_C);
//         reef.addOption("D", ReefLocs.REEF_D);
//         reef.addOption("E", ReefLocs.REEF_E);
//         reef.addOption("F", ReefLocs.REEF_F);
//         reef.addOption("G", ReefLocs.REEF_G);
//         reef.addOption("H", ReefLocs.REEF_H);
//         reef.addOption("I", ReefLocs.REEF_I);
//         reef.addOption("J", ReefLocs.REEF_J);
//         reef.addOption("K", ReefLocs.REEF_K);
//         reef.addOption("L", ReefLocs.REEF_L);
//     }

//     // add HP station options 
//     private static void addHPOptions(SendableChooser<HPStation> HP) {
//         HP.addOption("HP LEFT", HPStation.HP_LEFT);
//         HP.addOption("HP RIGHT", HPStation.HP_RIGHT);
//     }

//     // get Cycle Data of whats currently selected
//     public static ArrayList<AutonCycle> getAutonCycles() {
//         ArrayList<AutonCycle> cycles = new ArrayList<AutonCycle>();
//         int cycleCount = 0; // exists to handle the for loop if cyclesChooser.getselected() == null which happens RIGHT when the code starts
//         if (cyclesChooser.getSelected() != null) {
//             cycleCount = cyclesChooser.getSelected().m_cycles;
//         }

//         for (int i = 0; i < cycleCount; i++) {
//             AutonCycle currentCycle = autonFactory.new AutonCycle(
//                 hpToReefChoosers.get(i).getSelected(), 
//                 eleHeightChoosers.get(i).getSelected(), 
//                 reefToHPChoosers.get(i).getSelected()
//             );
//             cycles.add(currentCycle);
//         }

//         return cycles;
//     }

//     // updaters - called when a change in selection is detected by the listener
//     public static void updateNumCycles() {
//         m_cycles = cyclesChooser.getSelected();
//     }

//     public static void updateStartingHeight() {
//         startingHeight = startingHeightChooser.getSelected();
//     }

//     public static void updateStartingPosition() {
//         startingPosition = startingPositionChooser.getSelected();
//     }

//     public static void updateInitialScoringPosition() {
//         scoringPosition = firstScoringChooser.getSelected();
//     }

//     public static void updateInitalHPStation() {
//         hpStation = firstToHPStationChooser.getSelected();
//     }

//     public enum NumCycles {
//         CYCLE_1(1),
//         CYCLE_2(2),
//         CYCLE_3(3),
//         CYCLE_4(4);

//         public int m_cycles;

//         private NumCycles(int cycles){
//             m_cycles = cycles;
//         }

//         @Override
//         public String toString() {
//             return String.valueOf(m_cycles);
//         }
//     }
}