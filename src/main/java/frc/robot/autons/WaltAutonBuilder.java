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
    public static GenericEntry nte_midGOnly;
    
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
        // SmartDashboard.putData("Number of Cycles", cyclesChooser);
        // SmartDashboard.putData("Starting Position Chooser", startingPositionChooser);   
        // SmartDashboard.putData("Starting Elevator Height Chooser", startingHeightChooser);
        // SmartDashboard.putData("First Scoring Chooser", firstScoringChooser);
        // SmartDashboard.putData("First HP Station", firstToHPStationChooser);

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

        // nte_customAutonReady = Shuffleboard.getTab("AutonChooser")
        //         .add("Custom Auton Ready", false)
        //         .withWidget(BuiltInWidgets.kToggleSwitch)
        //         .getEntry();

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

        nte_midGOnly = Shuffleboard.getTab("AutonChooser")
                .add("Mid G Only", false)
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .getEntry();
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
