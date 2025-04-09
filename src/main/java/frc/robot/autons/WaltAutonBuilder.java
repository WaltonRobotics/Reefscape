package frc.robot.autons;

import java.util.ArrayList;

import choreo.auto.AutoFactory;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autons.TrajsAndLocs.HPReefPair;
import frc.robot.autons.TrajsAndLocs.HPStation;
import frc.robot.autons.TrajsAndLocs.ReefLocs;
import frc.robot.autons.TrajsAndLocs.StartingLocs;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Elevator.EleHeight;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Swerve;

public class WaltAutonBuilder {
    public static GenericEntry nte_autonEntry;
    public static GenericEntry nte_customAutonReady;
    public static GenericEntry nte_autonRobotPush;  // button to select if we are pushing another robot b4 starting path
    public static GenericEntry nte_clearAll;

    public static GenericEntry nte_autonReadyToGo;  // to let the user know that an auton is loaded
    public static GenericEntry nte_autonName;

    // DEFINE PRESET BUTTONS
    public static GenericEntry nte_taxiOnly;
    public static GenericEntry nte_rightThreePiece;
    public static GenericEntry nte_leftThreePiece;
    // public static GenericEntry nte_midGOnly;
    
    // ---- Initial
    // Define Initial Choosers
    public static SendableChooser<NumCycles> cyclesChooser = new SendableChooser<NumCycles>();
    public static SendableChooser<EleHeight> startingHeightChooser = new SendableChooser<EleHeight>();
    public static SendableChooser<StartingLocs> startingPositionChooser = new SendableChooser<StartingLocs>();
    public static SendableChooser<ReefLocs> firstScoringChooser = new SendableChooser<ReefLocs>();
    public static SendableChooser<HPStation> firstToHPStationChooser = new SendableChooser<HPStation>();

    // default, initial values
    public static NumCycles m_cycles = NumCycles.CYCLE_1;
    public static EleHeight startingHeight = EleHeight.L4;
    public static StartingLocs startingPosition = StartingLocs.RIGHT;
    public static ReefLocs scoringPosition = ReefLocs.REEF_A;
    public static HPStation hpStation = HPStation.HP_LEFT;

    static {
        SmartDashboard.putData("Number of Cycles", cyclesChooser);
        SmartDashboard.putData("Starting Position Chooser", startingPositionChooser);   
        SmartDashboard.putData("Starting Elevator Height Chooser", startingHeightChooser);
        SmartDashboard.putData("First Scoring Chooser", firstScoringChooser);
        SmartDashboard.putData("First HP Station", firstToHPStationChooser);

        nte_autonEntry = Shuffleboard.getTab("AutonChooser")
                .add("Make", false)
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .getEntry();

        nte_autonRobotPush = Shuffleboard.getTab("AutonChooser")
                .add("Push another robot?", false)
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .getEntry();

        nte_clearAll = Shuffleboard.getTab("AutonChooser")
                .add("CLEAR AUTON", false)
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .getEntry();

        nte_customAutonReady = Shuffleboard.getTab("AutonChooser")
                .add("Custom Auton Ready", false)
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .getEntry();

        nte_autonReadyToGo = Shuffleboard.getTab("AutonChooser")
                .add("Auton Ready", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .getEntry();

        nte_autonName = Shuffleboard.getTab("AutonChooser")
                .add("Auton", "No Auton Made")
                .withWidget(BuiltInWidgets.kTextView)
                .getEntry();

        // DEFINE PRESETS

        nte_taxiOnly = Shuffleboard.getTab("AutonChooser")
                .add("Taxi Only", false)
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .getEntry();

        nte_rightThreePiece = Shuffleboard.getTab("AutonChooser")
                .add("Right 3 Piece", false)
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .getEntry();
        
        nte_leftThreePiece = Shuffleboard.getTab("AutonChooser")
                .add("Left 3 Piece", false)
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .getEntry();

        nte_leftThreePiece = Shuffleboard.getTab("AutonChooser")
                .add("Mid 1 Piece", false)
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .getEntry();
        
    }

    // adds the options
    public static void configureFirstCycle() {
        // add cycle options
        cyclesChooser.addOption("Preload Only", NumCycles.PRELOAD_ONLY);
        cyclesChooser.addOption("1 Cycle", NumCycles.CYCLE_1);
        cyclesChooser.addOption("2 Cycle", NumCycles.CYCLE_2);
        cyclesChooser.addOption("3 Cycle", NumCycles.CYCLE_3);
        cyclesChooser.addOption("4 Cycle", NumCycles.CYCLE_4);

        // add Starting Position options
        startingPositionChooser.addOption("Left", StartingLocs.LEFT);
        startingPositionChooser.addOption("Middle Left", StartingLocs.MID_H);
        startingPositionChooser.addOption("Middle Right", StartingLocs.MID_G);
        startingPositionChooser.addOption("Right", StartingLocs.RIGHT);
        startingPositionChooser.addOption("Super Left", StartingLocs.SUPER_LEFT);
        startingPositionChooser.addOption("Super Right", StartingLocs.SUPER_RIGHT);

        // changing the starting position AFFECTS HERE - see disable periodic
        if (startingPosition.equals(StartingLocs.LEFT)) {
            for (ReefLocs loc : TrajsAndLocs.ReefLocs.OptimalLeftStartCycles) {
                firstScoringChooser.addOption(loc.name(), loc);
            }
        } else if (startingPosition.equals(StartingLocs.SUPER_LEFT)) {
            for (ReefLocs loc : TrajsAndLocs.ReefLocs.OptimalSuperLeftStartCycles) {
                firstScoringChooser.addOption(loc.name(), loc);
            }
        } else if (startingPosition.equals(StartingLocs.MID_G)) {
            for (ReefLocs loc : TrajsAndLocs.ReefLocs.OptimalMidGStartCycles) {
                firstScoringChooser.addOption(loc.name(), loc);
            }
        } else if (startingPosition.equals(StartingLocs.MID_H)) {
            for (ReefLocs loc : TrajsAndLocs.ReefLocs.OptimalMidHStartCycles) {
                firstScoringChooser.addOption(loc.name(), loc);
            }
        } else if (startingPosition.equals(StartingLocs.RIGHT)) {
            for (ReefLocs loc : TrajsAndLocs.ReefLocs.OptimalRightStartCycles) {
                firstScoringChooser.addOption(loc.name(), loc);
            }
        } else {
            for (ReefLocs loc : TrajsAndLocs.ReefLocs.OptimalSuperRightStartCycles) {
                firstScoringChooser.addOption(loc.name(), loc);
            }
        }

        // add Starting Height options
        addEleOptions(startingHeightChooser);

        // add HP station options
        addHPOptions(firstToHPStationChooser);
    }

    // ---- Cycles
    // define cycle choosers
    public static ArrayList<SendableChooser<EleHeight>> eleHeightChoosers = new ArrayList<SendableChooser<EleHeight>>();
    public static ArrayList<SendableChooser<ReefLocs>> hpToReefChoosers = new ArrayList<SendableChooser<ReefLocs>>();
    public static ArrayList<SendableChooser<HPStation>> reefToHPChoosers = new ArrayList<SendableChooser<HPStation>>();

    // TODO: MAYBE add method that closes unnecessary sendable choosers (when numCycles Changes)

    // adds choosers based on number of cycles selected
    public static void configureCycles() {
        System.out.println("adding choosers");
        for (int i = 0; i < m_cycles.m_cycles; i++) {
            eleHeightChoosers.add(new SendableChooser<EleHeight>());
            hpToReefChoosers.add(new SendableChooser<ReefLocs>());
            reefToHPChoosers.add(new SendableChooser<HPStation>());

            addEleOptions(eleHeightChoosers.get(i));
            addReefOptions(hpToReefChoosers.get(i));
            addHPOptions(reefToHPChoosers.get(i));

            SmartDashboard.putData("HP To Reef Chooser - Cycle " + (i + 1), hpToReefChoosers.get(i));
            SmartDashboard.putData("Elevator Height - Cycle " + (i + 1), eleHeightChoosers.get(i));   
            SmartDashboard.putData("Reef to HP Chooser - Cycle " + (i + 1), reefToHPChoosers.get(i));
        }
    }

    // ---- add Options
    // add elevator height options
    private static void addEleOptions(SendableChooser<EleHeight> eleHeight) {
        eleHeight.addOption("L1", EleHeight.L1);
        eleHeight.addOption("L2", EleHeight.L2);
        eleHeight.addOption("L3", EleHeight.L3);
        eleHeight.addOption("L4", EleHeight.L4);
    }

    // add reef options
    private static void addReefOptions(SendableChooser<ReefLocs> reef) {
        reef.addOption("A", ReefLocs.REEF_A);
        reef.addOption("B", ReefLocs.REEF_B);
        reef.addOption("C", ReefLocs.REEF_C);
        reef.addOption("D", ReefLocs.REEF_D);
        reef.addOption("E", ReefLocs.REEF_E);
        reef.addOption("F", ReefLocs.REEF_F);
        reef.addOption("G", ReefLocs.REEF_G);
        reef.addOption("H", ReefLocs.REEF_H);
        reef.addOption("I", ReefLocs.REEF_I);
        reef.addOption("J", ReefLocs.REEF_J);
        reef.addOption("K", ReefLocs.REEF_K);
        reef.addOption("L", ReefLocs.REEF_L);
    }

    // add HP station options 
    private static void addHPOptions(SendableChooser<HPStation> HP) {
        HP.addOption("HP LEFT", HPStation.HP_LEFT);
        HP.addOption("HP RIGHT", HPStation.HP_RIGHT);
    }

    // get Cycle Data of whats currently selected
    public static ArrayList<ReefLocs> getCycleScoringLocs() {
        ArrayList<ReefLocs> reefLocs = new ArrayList<ReefLocs>();
        int cycleCount = 0; // exists to handle the for loop if cyclesChooser.getselected() == null which happens RIGHT when the code starts
        if (cyclesChooser.getSelected() != null) {
            cycleCount = cyclesChooser.getSelected().m_cycles;
        }

        // add preload
        reefLocs.add(scoringPosition);

        for (int i = 0; i < cycleCount; i++) {
            reefLocs.add(hpToReefChoosers.get(i).getSelected());
        }

        return reefLocs;
    }

    public static ArrayList<EleHeight> getCycleEleHeights() {
        ArrayList<EleHeight> eleHeights = new ArrayList<EleHeight>();
        int cycleCount = 0; // exists to handle the for loop if cyclesChooser.getselected() == null which happens RIGHT when the code starts
        if (cyclesChooser.getSelected() != null) {
            cycleCount = cyclesChooser.getSelected().m_cycles;
        }

        // add preload
        eleHeights.add(startingHeight);

        for (int i = 0; i < cycleCount; i++) {
            eleHeights.add(eleHeightChoosers.get(i).getSelected());
        }

        return eleHeights;
    }

    public static ArrayList<HPStation> getCycleHPStations() {
        ArrayList<HPStation> hpStations = new ArrayList<HPStation>();
        int cycleCount = 0; // exists to handle the for loop if cyclesChooser.getselected() == null which happens RIGHT when the code starts
        if (cyclesChooser.getSelected() != null) {
            cycleCount = cyclesChooser.getSelected().m_cycles;
        }

        // add preload
        hpStations.add(hpStation);

        for (int i = 0; i < cycleCount; i++) {
            hpStations.add(reefToHPChoosers.get(i).getSelected());
        }

        return hpStations;
    }

    // updaters - called when a change in selection is detected by the listener
    public static void updateNumCycles() {
        m_cycles = cyclesChooser.getSelected();
    }

    public static void updateStartingHeight() {
        startingHeight = startingHeightChooser.getSelected();
    }

    public static void updateStartingPosition() {
        startingPosition = startingPositionChooser.getSelected();
    }

    public static void updateInitialScoringPosition() {
        scoringPosition = firstScoringChooser.getSelected();
    }

    public static void updateInitalHPStation() {
        hpStation = firstToHPStationChooser.getSelected();
    }

    public enum NumCycles {
        PRELOAD_ONLY(0),
        CYCLE_1(1),
        CYCLE_2(2),
        CYCLE_3(3),
        CYCLE_4(4);

        public int m_cycles;

        private NumCycles(int cycles){
            m_cycles = cycles;
        }

        @Override
        public String toString() {
            return String.valueOf(m_cycles);
        }
    }
}
