package frc.robot.autons;

import java.util.ArrayList;
import java.util.Optional;
import java.util.Set;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    public static enum CycleCount {
        ZERO_TRUE_CYCLES(0, "ZERO_TRUE_CYCLES"),
        ONE_CYCLE(1, "ONE_CYCLE"),
        TWO_CYCLES(2, "TWO CYCLES"),
        THREE_CYCLES(3, "THREE CYCLES"),
        FOUR_CYCLES(4, "FOUR CYCLES"),
        FIVE_CYCLES(5, "FIVE CYCLES");
        // WARNING: Constants.AutonChooserK.maxAutonCycleCount determines the maximum number of cycles.
        // do not add anything here without updating that value so that network tables understands what it's doing

        public final int m_count;
        public final String m_name;

        private CycleCount(int count, String name) {
            m_count = count;
            m_name = name;
        }
    }

    // THIS DOES NOT MODIFY ANYTHING RELATED TO NETWORK TABLES
    // this solely tells the auton generation to consider this the auton cycle count and ignore any after that
    private static final SendableChooser<CycleCount> cycleCountChooser = new SendableChooser<>();

    // declare and define SendableChoosers for special starting cycle
    private static SendableChooser<StartingLocs> startingPositionChooser = new SendableChooser<>();
    private static SendableChooser<ReefLocs> firstScoreChooser = new SendableChooser<>();
    private static SendableChooser<EleHeight> firstScoreHeightChooser = new SendableChooser<>();
    private static SendableChooser<HPStation> firstHPStationChooser = new SendableChooser<>();

    // declare and define SendableChoosers for cycles
    private static final ArrayList<SendableChooser<ReefLocs>> hpToReefChoosers = new ArrayList<>();
    private static final ArrayList<SendableChooser<HPStation>> reefToHPChoosers = new ArrayList<>();
    private static final ArrayList<SendableChooser<EleHeight>> scoreHeightChoosers = new ArrayList<>();

    public static void init() {
        cycleCountChooser.onChange(AutonChooser::updateChoices);
        cycleCountChooser.setDefaultOption("Zero True Cycles", CycleCount.ZERO_TRUE_CYCLES);
        cycleCountChooser.addOption("One Cycle", CycleCount.ONE_CYCLE);
        cycleCountChooser.addOption("Two Cycles", CycleCount.TWO_CYCLES);
        cycleCountChooser.addOption("Three Cycles", CycleCount.THREE_CYCLES);
        cycleCountChooser.addOption("Four Cycles", CycleCount.FOUR_CYCLES);
        cycleCountChooser.addOption("Five Cycles", CycleCount.FIVE_CYCLES);
        SmartDashboard.putData("Cycle Count", cycleCountChooser);

        // staring position chooser
        startingPositionChooser.onChange(chooser -> updateChoices(cycleCountChooser.getSelected()));
        startingPositionChooser.setDefaultOption("Mid", StartingLocs.MID);
        startingPositionChooser.addOption("Left", StartingLocs.LEFT);
        startingPositionChooser.addOption("Right", StartingLocs.RIGHT);
        SmartDashboard.putData("Starting Position", startingPositionChooser);

        basicChooserConfig();
        SmartDashboard.updateValues();
    }

    public static Command forceUpdateDEBUG() {
        return Commands.runOnce(() -> updateChoices(cycleCountChooser.getSelected()));
    }

    /**
     * Think about whether you actually need to use this!!! i'm just doing debug stuff iwth it
     */
    public static Command getChoiceNameDEBUG() {
        String value;
        if (firstScoreChooser.getSelected() != null) {
            value = "value: " + firstScoreChooser.getSelected().name();
        } else {
            value = "value is null";
        }
        return Commands.print(value);
    }

    /**
     * Runs and puts all empty choosers. Do not call this outside of initialization above
     */
    private static void basicChooserConfig() {
        firstScoreChooser.onChange(chooser -> updateChoices(cycleCountChooser.getSelected()));
        SmartDashboard.putData("First Score Location", firstScoreChooser);
        firstHPStationChooser.onChange(chooser -> updateChoices(cycleCountChooser.getSelected()));
        SmartDashboard.putData("First Human Station Location", firstHPStationChooser);
        firstScoreHeightChooser.onChange(chooser -> updateChoices(cycleCountChooser.getSelected()));
        SmartDashboard.putData("First Score Height", firstScoreHeightChooser);
        // hpToReefChoosers
        for (int i = 0; i < AutonChooserK.maxAutonCycleCount; i++) {
            SendableChooser<ReefLocs> currentChooser = new SendableChooser<ReefLocs>();
            currentChooser.onChange(chooser -> updateChoices(cycleCountChooser.getSelected()));
            hpToReefChoosers.add(currentChooser);
            SmartDashboard.putData("Cycle " + Integer.toString(i + 1) + " Scoring Location Chooser", currentChooser);
        }
        // reefToHPChoosers
        for (int i = 0; i < AutonChooserK.maxAutonCycleCount; i++) {
            SendableChooser<HPStation> currentChooser = new SendableChooser<HPStation>();
            currentChooser.onChange(chooser -> updateChoices(cycleCountChooser.getSelected()));
            reefToHPChoosers.add(currentChooser);
            SmartDashboard.putData("Cycle " + Integer.toString(i + 1) + " HP Station Chooser", currentChooser);
        }
        // scoreHeightChoosers
        for (int i = 0; i < AutonChooserK.maxAutonCycleCount; i++) {
            SendableChooser<EleHeight> currentChooser = new SendableChooser<EleHeight>();
            currentChooser.onChange(chooser -> updateChoices(cycleCountChooser.getSelected()));
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
        System.out.println("run update chooser");

        firstScoreChooser.close();
        Optional<ArrayList<ReefLocs>> optimalStartScoringLocsOptional = getOptimalReefLocs(startingPositionChooser.getSelected());
        // this should only occur if startingPositionChooser.getSelected() == null, meaning there is no starting location selected (somehow)
        // this should not be possible for starting location but it will be possible a little farther in for other values so
        // the method needs to account for it
        if (optimalStartScoringLocsOptional.isEmpty()) {
            endAutonUpdate("No starting location available", 0, UpdateStages.REEF_LOCATION);
            return;
        }
        ArrayList<ReefLocs> optimalStartScoringLocs = optimalStartScoringLocsOptional.get();
        SendableChooser<ReefLocs> newFirstScoreChooser = new SendableChooser<ReefLocs>();
        for (ReefLocs reefLoc : optimalStartScoringLocs) {
            newFirstScoreChooser.addOption(reefLoc.name(), reefLoc);
        }
        firstScoreChooser = newFirstScoreChooser;
        SmartDashboard.putData("First Score Location", firstScoreChooser);

        firstHPStationChooser.close();
        Optional<ArrayList<HPStation>> optimalStartHPStationsOptional = getOptimalHPStations(firstScoreChooser.getSelected());
        if (optimalStartHPStationsOptional.isEmpty()) {
            endAutonUpdate("No first reef location available", 0, UpdateStages.HP_LOCATION);
            return;
        }
        ArrayList<HPStation> optimalStartHPStations = optimalStartHPStationsOptional.get();
        SendableChooser<HPStation> newFirstHPStationChooser = new SendableChooser<HPStation>();
        for (HPStation hpStation : optimalStartHPStations) {
            newFirstHPStationChooser.addOption(hpStation.name(), hpStation);
        }
        firstHPStationChooser = newFirstHPStationChooser;
        SmartDashboard.putData("First HP Location", firstHPStationChooser);

        SmartDashboard.updateValues();
    }

    /**
     * 
     * @param failureMessage
     * @param endCycle Should be [0, kMaxCycleCount]
     * @param endStage Notes what stage of AutonUpdate this ended during
     */
    private static void endAutonUpdate(String failureMessage, int endCycle, UpdateStages endStage) {
        SmartDashboard.updateValues();
        System.out.println("[AutonChooser]: updateChoices end with message: " + failureMessage);
        // this flag simply exists to ensure that we do complete cycles after account for the incomplete cycle
        boolean accountedForPartialUpdate = false;
        if (endCycle == 0) {
            switch (endStage) {
                // note lack of breaks - this is intentional so it executes everything after
                case STARTING_LOCATION:
                    accountedForPartialUpdate = true;

                    startingPositionChooser.close();
                    startingPositionChooser = new SendableChooser<StartingLocs>();
                    startingPositionChooser.onChange(chooser -> updateChoices(cycleCountChooser.getSelected()));

                    startingPositionChooser.setDefaultOption("Mid", StartingLocs.MID);
                    startingPositionChooser.addOption("Left", StartingLocs.LEFT);
                    startingPositionChooser.addOption("Right", StartingLocs.RIGHT);
                    SmartDashboard.putData("Starting Position", startingPositionChooser);
                case REEF_LOCATION:
                    accountedForPartialUpdate = true;

                    firstScoreChooser.close();
                    firstScoreChooser = new SendableChooser<ReefLocs>();
                    firstScoreChooser.onChange(chooser -> updateChoices(cycleCountChooser.getSelected()));

                    SmartDashboard.putData("First Score Location", firstScoreChooser);
                case HP_LOCATION:
                    accountedForPartialUpdate = true;

                    firstHPStationChooser.close();
                    firstHPStationChooser = new SendableChooser<HPStation>();
                    firstHPStationChooser.onChange(chooser -> updateChoices(cycleCountChooser.getSelected()));

                    SmartDashboard.putData("First Human Station Location", firstHPStationChooser);
                case SCORE_HEIGHT:
                    accountedForPartialUpdate = true;

                    firstScoreHeightChooser.close();
                    firstScoreHeightChooser = new SendableChooser<EleHeight>();
                    firstScoreHeightChooser.onChange(chooser -> updateChoices(cycleCountChooser.getSelected()));

                    SmartDashboard.putData("First Score Height", firstScoreHeightChooser);
            }

            // BE REAL CAREFUL ACCESSING endCycle - IT HAS TO BE MODIFIED TO 1 HERE TO AVOID INDEX OUT OF BOUNDS.
            // TO ENSURE ACCURATE ACCESS OF endCycle ACCESS IT BEFORE THIS POINT
            endCycle = 1;
        }

        for (int cycleNum = endCycle; cycleNum < AutonChooserK.maxAutonCycleCount; cycleNum++) {
            // having two numbers for this makes life just a little easier
            int actualIndex = cycleNum - 1;
            // this makes sure we do full cycles every cycle after accounting for the one that completed partially
            if (endStage == UpdateStages.REEF_LOCATION || accountedForPartialUpdate) {
                accountedForPartialUpdate = true;

                hpToReefChoosers.remove(actualIndex).close();
                SendableChooser<ReefLocs> currentChooser = new SendableChooser<ReefLocs>();
                currentChooser.onChange(chooser -> updateChoices(cycleCountChooser.getSelected()));
                hpToReefChoosers.add(actualIndex, currentChooser);

                SmartDashboard.putData("Cycle " + Integer.toString(cycleNum) + " Scoring Location Chooser", currentChooser);
            }
            if (endStage == UpdateStages.HP_LOCATION || accountedForPartialUpdate) {
                accountedForPartialUpdate = true;

                reefToHPChoosers.remove(actualIndex).close();
                SendableChooser<HPStation> currentChooser = new SendableChooser<HPStation>();
                currentChooser.onChange(chooser -> updateChoices(cycleCountChooser.getSelected()));
                reefToHPChoosers.add(actualIndex, currentChooser);
            }
            if (endStage == UpdateStages.SCORE_HEIGHT || accountedForPartialUpdate) {
                accountedForPartialUpdate = true;

                scoreHeightChoosers.remove(actualIndex).close();
                SendableChooser<EleHeight> currentChooser = new SendableChooser<EleHeight>();
                currentChooser.onChange(chooser -> updateChoices(cycleCountChooser.getSelected()));
                scoreHeightChoosers.add(actualIndex, currentChooser);
            }
        }

        SmartDashboard.updateValues();
    }

    private static Optional<ArrayList<ReefLocs>> getOptimalReefLocs(StartingLocs startingLocation) {
        // this will likely come straight from SendableChooser so you have to be prepared
        if (startingLocation == null) {
            return Optional.empty();
        }
        return Optional.of(ReefLocs.optimalPathsByMatchStartLocation.get(startingLocation));
    }

    private static Optional<ArrayList<ReefLocs>> getOptimalReefLocs(HPStation hpStation) {
        // this will likely come straight from SendableChooser so you have to be prepared
        if (hpStation == null) {
            return Optional.empty();
        }
        return Optional.of(ReefLocs.optimalPathsByHPStation.get(hpStation));
    }

    private static Optional<ArrayList<HPStation>> getOptimalHPStations(ReefLocs reefLocation) {
        if (reefLocation == null) {
            return Optional.empty();
        }
        Set<HPStation> allHPStationValues = ReefLocs.optimalPathsByHPStation.keySet();
        ArrayList<HPStation> outputList = new ArrayList<HPStation>();
        // TODO: just be warned that if this doesn't function look here - not sure using a foreach loop in this manner is perfect
        for (HPStation hpStation : allHPStationValues) {
            if (ReefLocs.optimalPathsByHPStation.get(hpStation).contains(reefLocation)) {
                outputList.add(hpStation);
            }
        }

        return Optional.of(outputList);
    }

    private enum UpdateStages {
        STARTING_LOCATION(-1, "STARTING_LOCATION"),
        REEF_LOCATION(0, "REEF_LOCATION"),
        HP_LOCATION(1, "HP_LOCATION"),
        SCORE_HEIGHT(2, "SCORE_HEIGHT");

        public final int m_index;
        public final String m_name;

        private UpdateStages(int index, String name) {
            m_index = index;
            m_name = name;
        }
    }
}
