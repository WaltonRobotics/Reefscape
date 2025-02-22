package frc.robot.autons;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutonChooserK;
import frc.robot.autons.TrajsAndLocs.HPStation;
import frc.robot.autons.TrajsAndLocs.ReefLocs;
import frc.robot.autons.TrajsAndLocs.StartingLocs;
import frc.robot.autons.WaltAutonFactory.AutonCycle;
import frc.robot.subsystems.Elevator.EleHeight;

public class AutonChooser {
    /**
     * This determines how many true cycles to account for.
     * A cycle consists of a movement from the Human Player Station to the Reef and back.
     */
    private static enum CycleCount {
        ZERO_TRUE_CYCLES(0),
        ONE_CYCLE(1),
        TWO_CYCLES(2),
        THREE_CYCLES(3),
        FOUR_CYCLES(4),
        FIVE_CYCLES(5);
        // WARNING: Constants.AutonChooserK.maxAutonCycleCount determines the maximum number of cycles.
        // do not add anything here without updating that value so that network tables understands what it's doing

        public final int m_count;

        private CycleCount(int count) {
            m_count = count;
        }
    }

    // THIS DOES NOT MODIFY ANYTHING RELATED TO NETWORK TABLES
    // this solely tells the auton generation to consider this the auton cycle count and ignore any after that
    private static final SendableChooser<CycleCount> cycleCountChooser = new SendableChooser<>();

    // declare and define SendableChoosers for special starting cycle
    private static SendableChooser<StartingLocs> startingPositionChooser = new SendableChooser<>();
    private static SendableChooser<ReefLocs> firstScoreChooser = new SendableChooser<>();
    private static SendableChooser<EleHeight> firstScoreHeight = new SendableChooser<>();
    private static SendableChooser<HPStation> firstHPStationChooser = new SendableChooser<>();

    // declare and define SendableChoosers for cycles
    private static final ArrayList<SendableChooser<ReefLocs>> hpToReefChoosers = new ArrayList<>();
    private static final ArrayList<SendableChooser<HPStation>> reefToHPChoosers = new ArrayList<>();
    private static final ArrayList<SendableChooser<EleHeight>> scoreHeightChoosers = new ArrayList<>();

    static {
        // configure SendableChoosers
        // STARTING CHOOSERS
        // cycle count chooser
        cycleCountChooser.setDefaultOption("Zero True Cycles", CycleCount.ZERO_TRUE_CYCLES);
        cycleCountChooser.addOption("One Cycle", CycleCount.ONE_CYCLE);
        cycleCountChooser.addOption("Two Cycles", CycleCount.TWO_CYCLES);
        cycleCountChooser.addOption("Three Cycles", CycleCount.THREE_CYCLES);
        cycleCountChooser.addOption("Four Cycles", CycleCount.FOUR_CYCLES);
        cycleCountChooser.addOption("Five Cycles", CycleCount.FIVE_CYCLES);
        SmartDashboard.putData("Cycle Count", cycleCountChooser);

        // staring position chooser
        startingPositionChooser.setDefaultOption("Mid", StartingLocs.MID);
        startingPositionChooser.addOption("Left", StartingLocs.LEFT);
        startingPositionChooser.addOption("Right", StartingLocs.RIGHT);
        SmartDashboard.putData("Starting Position", startingPositionChooser);

        basicChooserConfig();
        cycleCountChooser.onChange(AutonChooser::updateChoices);
    }

    /**
     * Runs and puts all empty choosers. Do not call this outside of initialization above
     */
    private static void basicChooserConfig() {
        SmartDashboard.putData("First Score Location", firstScoreChooser);
        SmartDashboard.putData("First Score Height", firstScoreHeight);
        SmartDashboard.putData("First Human Station Location", firstHPStationChooser);
        // hpToReefChoosers
        for (int i = 0; i < AutonChooserK.maxAutonCycleCount; i++) {
            SendableChooser<ReefLocs> currentChooser = new SendableChooser<ReefLocs>();
            hpToReefChoosers.add(currentChooser);
            SmartDashboard.putData("Cycle " + Integer.toString(i + 1) + " Scoring Location Chooser", currentChooser);
        }
        // reefToHPChoosers
        for (int i = 0; i < AutonChooserK.maxAutonCycleCount; i++) {
            SendableChooser<HPStation> currentChooser = new SendableChooser<HPStation>();
            reefToHPChoosers.add(currentChooser);
            SmartDashboard.putData("Cycle " + Integer.toString(i + 1) + " HP Station Chooser", currentChooser);
        }
        // scoreHeightChoosers
        for (int i = 0; i < AutonChooserK.maxAutonCycleCount; i++) {
            SendableChooser<EleHeight> currentChooser = new SendableChooser<EleHeight>();
            scoreHeightChoosers.add(currentChooser);
            SmartDashboard.putData("Cycle " + Integer.toString(i + 1) + " Score Height Chooser", currentChooser);
        }
    }

    public static StartingLocs getChosenStart() {
        return null;
    }

    public static ReefLocs getChosenFirstReef() {
        return null;
    }

    public static EleHeight getStartingHeight() {
        return null;
    }

    public static HPStation getChosenFirstHP() {
        return null;
    }

    public static ArrayList<AutonCycle> getCycles() {
        return null;
    }
    
    /**
     * This should be bound to all our choosers so that when any of them change we redetermine which options are valid. 
     * This closes all our choosers and redetermines which options are valid.
     * @param cycleCount
     */
    private static void updateChoices(CycleCount cycleCount) {
        firstScoreChooser.close();
        Optional<ArrayList<ReefLocs>> optimalStartScoringLocsOptional = TrajsAndLocs.getOptimalStartCycles(startingPositionChooser.getSelected());
        if (optimalStartScoringLocsOptional.isEmpty()) {closeAndReopen(0); return;}
        ArrayList<ReefLocs> optimalStartScoringLocs = optimalStartScoringLocsOptional.get();
        SendableChooser<ReefLocs> newFirstScoreChooser = new SendableChooser<ReefLocs>();
        for (ReefLocs reefLoc : optimalStartScoringLocs) {
            newFirstScoreChooser.addOption(reefLoc.name(), reefLoc);
        }
        firstScoreChooser = newFirstScoreChooser;
        SmartDashboard.putData("First Score Location", firstScoreChooser);

        firstHPStationChooser.close();
        Optional<ArrayList<HPStation>> optimalStartHPStationsOptional = TrajsAndLocs.getOptimalHPCycles(startingPositionChooser.getSelected());
        if (optimalStartHPStationsOptional.isEmpty()) {closeAndReopen(0); return;}
        ArrayList<ReefLocs> optimalStartHPStations = optimalStartHPStationsOptional.get();
        SendableChooser<ReefLocs> newFirstHPStationChooser = new SendableChooser<ReefLocs>();
        for (ReefLocs reefLoc : optimalStartHPStations) {
            newFirstHPStationChooser.addOption(reefLoc.name(), reefLoc);
        }
        firstHPStationChooser = newFirstHPStationChooser;
        SmartDashboard.putData("First Score Location", firstHPStationChooser);

        SmartDashboard.updateValues();
    }

    private static void closeAndReopen(int cycleIndicesToClose) {
        if (cycleIndicesToClose < 1) {

        }
    }

    /**
     * @param startingLocation Will likely come directly from SendableChoosers, so it will be prepared to receive null
     * @return
     */
    private static Optional<ArrayList<ReefLocs>> getOptimalStartScoringLocations(StartingLocs startingLocation) {
        if (startingLocation == null) {
            return Optional.empty();
        }
        ArrayList<ReefLocs> rtn = new ArrayList<ReefLocs>();
        for ()
    }

    private static Optional<List<ReefLocs>> getAvailableReefLocs(StartingLocs startingLocation) {
        // this will likely come straight from SendableChooser so you have to be prepared
        if (startingLocation == null) {
            return Optional.empty();
        }
        return Optional.of(ReefLocs.optimalPathsByMatchStartLocation.get(startingLocation));
    }

    private static Optional<List<ReefLocs>> getAvailableReefLocs(HPStation hpStation) {
        // this will likely come straight from SendableChooser so you have to be prepared
        if (hpStation == null) {
            return Optional.empty();
        }
        return Optional.of(ReefLocs.optimalPathsByHPStation.get(hpStation));
    }

    private static Optional<List<HPStation>> getAvailableHPStation(ReefLocs reefLocation) {
        if (reefLocation == null) {
            return Optional.empty();
        }
        Set<HPStation> allHPStationValues = ReefLocs.optimalPathsByHPStation.keySet();
        for (Iterator<HPStation> iter = allHPStationValues.iterator(); iter.hasNext(); ) {
            HPStation currentHPStationValue = iter.next();
            
        }
    }
}
