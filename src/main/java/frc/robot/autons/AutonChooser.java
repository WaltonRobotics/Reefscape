package frc.robot.autons;

import java.util.ArrayList;
import java.util.EnumMap;
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
    private static final ArrayList<SendableChooser<ReefLocs>> scoreLocationChoosers = new ArrayList<>();
    private static final ArrayList<SendableChooser<HPStation>> hpStationChoosers = new ArrayList<>();
    private static final ArrayList<SendableChooser<EleHeight>> scoreHeightChoosers = new ArrayList<>();

    private static EnumMap<UpdateStages, String> updateStageToKeyText = new EnumMap<>(UpdateStages.class);

    // ------------------
    // ---- MAIN API ----
    // ------------------

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

    // ---------------
    // ---- DEBUG ----
    // ---------------

    public static Command forceUpdateDEBUG() {
        return Commands.runOnce(() -> updateChoices(cycleCountChooser.getSelected(), 0, UpdateStages.STARTING_LOCATION));
    }
    
    // ------------------------
    // ---- PROGRAM DEPTHS ----
    // ------------------------

    /**
     * Should run at program init
     */
    public static void init() {
        configGeneratedNTKeys();

        cycleCountChooser.addOption("One Cycle", CycleCount.ONE_CYCLE);
        cycleCountChooser.addOption("Two Cycles", CycleCount.TWO_CYCLES);
        cycleCountChooser.addOption("Three Cycles", CycleCount.THREE_CYCLES);
        cycleCountChooser.addOption("Four Cycles", CycleCount.FOUR_CYCLES);
        cycleCountChooser.addOption("Five Cycles", CycleCount.FIVE_CYCLES);
        cycleCountChooser.addOption("Zero True Cycles", CycleCount.ZERO_TRUE_CYCLES);
        cycleCountChooser.setDefaultOption(AutonChooserK.kDefaultNullMessage, null);
        cycleCountChooser.onChange(choice -> updateChoices(cycleCountChooser.getSelected(), 0, UpdateStages.STARTING_LOCATION));
        SmartDashboard.putData("Cycle Count", cycleCountChooser);

        // staring position chooser
        startingPositionChooser.onChange(choice -> updateChoices(cycleCountChooser.getSelected(), 0, UpdateStages.STARTING_LOCATION));
        startingPositionChooser.addOption("Left", StartingLocs.LEFT);
        startingPositionChooser.addOption("Right", StartingLocs.RIGHT);
        startingPositionChooser.addOption("Mid", StartingLocs.MID);
        cycleCountChooser.setDefaultOption(AutonChooserK.kDefaultNullMessage, null);
        SmartDashboard.putData(AutonChooserK.kStartingPosChooserKey, startingPositionChooser);

        // populate arraylists
        for (int i = 0; i < AutonChooserK.kMaxAutonCycleCount; i++) {
            SendableChooser<ReefLocs> scoreLocationChooser = new SendableChooser<ReefLocs>();
            scoreLocationChoosers.add(scoreLocationChooser);
            scoreLocationChoosers.get(i).onChange(choice -> printNullOptionChanged());
            SmartDashboard.putData(generateKey(i+1, UpdateStages.SCORE_LOCATION), scoreLocationChoosers.get(i));
            
            SendableChooser<HPStation> hpStationChooser = new SendableChooser<HPStation>();
            hpStationChoosers.add(hpStationChooser);
            hpStationChoosers.get(i).onChange(choice -> printNullOptionChanged());
            SmartDashboard.putData(generateKey(i+1, UpdateStages.HP_STATION_LOCATION), hpStationChoosers.get(i));

            SendableChooser<EleHeight> scoreHeightChooser = new SendableChooser<EleHeight>();
            scoreHeightChoosers.add(scoreHeightChooser);
            scoreHeightChoosers.get(i).onChange(choice -> printNullOptionChanged());
            SmartDashboard.putData(generateKey(i+1, UpdateStages.ELE_HEIGHT), scoreHeightChoosers.get(i));
        }

        // this should initialize things
        resetOptions(0, UpdateStages.SCORE_LOCATION);
    }

    /**
     * This resets all the choices beyond
     * @param endCycleNum 0 corresponds to a failure before the beginning of the first true cycle
     * @param resetPastInclusive This is the stage of update that the update ended during
     */
    private static void resetOptions(int endCycleNum, UpdateStages resetPastInclusive) {
        System.out.println("reset options begins with endcyclenum " + endCycleNum + " and resetPastInclusive " + resetPastInclusive.m_name);
        boolean accountedForPartialCycle = false;
        for (int cycleNum = endCycleNum; cycleNum <= AutonChooserK.kMaxAutonCycleCount; cycleNum++) {
            int realCycleIndex = cycleNum - 1;
            if (cycleNum == 0) {
                switch (resetPastInclusive) {
                    case STARTING_LOCATION:
                        System.out.println("It's time to be confused how resetOPtions found cycle 0 starting_locaiton reset");
                    case SCORE_LOCATION:
                        accountedForPartialCycle = true;
                        firstScoreChooser.close();

                        firstScoreChooser = new SendableChooser<ReefLocs>();
                        firstScoreChooser.onChange(choice -> printNullOptionChanged());
                        SmartDashboard.putData(AutonChooserK.kFirstScoreLocChooserKey, firstScoreChooser);
                    case HP_STATION_LOCATION:
                        accountedForPartialCycle = true;
                        firstHPStationChooser.close();

                        firstHPStationChooser = new SendableChooser<HPStation>();
                        firstHPStationChooser.onChange(choice -> printNullOptionChanged());
                        SmartDashboard.putData(AutonChooserK.kFirstHPStationChooserKey, firstHPStationChooser);
                    case ELE_HEIGHT:
                        accountedForPartialCycle = true;
                        firstScoreHeightChooser.close();

                        firstScoreHeightChooser = new SendableChooser<EleHeight>();
                        firstScoreHeightChooser.onChange(choice -> printNullOptionChanged());
                        SmartDashboard.putData(AutonChooserK.kFirstScoreHeightChooserKey, firstScoreHeightChooser);
                    default:
                        break;
                }
            } else {
                // handle cycles properly if the cycleNum is something other than the starting cycle (0)
                if ((UpdateStages.SCORE_LOCATION == resetPastInclusive && endCycleNum == cycleNum) || accountedForPartialCycle) {
                    accountedForPartialCycle = true;
                    scoreLocationChoosers.remove(realCycleIndex).close();

                    SendableChooser<ReefLocs> scoreLocationChooser = new SendableChooser<ReefLocs>();
                    scoreLocationChoosers.add(realCycleIndex, scoreLocationChooser);
                    scoreLocationChoosers.get(realCycleIndex).onChange(choice -> printNullOptionChanged());
                    SmartDashboard.putData(scoreLocationChoosers.get(realCycleIndex));
                }
                if ((UpdateStages.HP_STATION_LOCATION == resetPastInclusive && endCycleNum == cycleNum) || accountedForPartialCycle) {
                    accountedForPartialCycle = true;
                    hpStationChoosers.remove(realCycleIndex).close();

                    SendableChooser<HPStation> hpStationChooser = new SendableChooser<HPStation>();
                    hpStationChoosers.add(realCycleIndex, hpStationChooser);
                    hpStationChoosers.get(realCycleIndex).onChange(choice -> printNullOptionChanged());
                    SmartDashboard.putData(scoreLocationChoosers.get(realCycleIndex));
                }
                if ((UpdateStages.ELE_HEIGHT == resetPastInclusive && endCycleNum == cycleNum) || accountedForPartialCycle) {
                    accountedForPartialCycle = true;
                    scoreHeightChoosers.remove(realCycleIndex).close();

                    SendableChooser<EleHeight> scoreHeightChooser = new SendableChooser<EleHeight>();
                    scoreHeightChoosers.add(realCycleIndex, scoreHeightChooser);
                    scoreHeightChoosers.get(realCycleIndex).onChange(choice -> printNullOptionChanged());
                    SmartDashboard.putData(scoreLocationChoosers.get(realCycleIndex));
                }
            }
        }
    }

    private static void updateChoices(CycleCount cycleCountChoice, int cycleNum, UpdateStages stageCalledFrom) {
        boolean passedDangerPoint = false;
        if (cycleCountChoice == null) {
            System.out.println("cycle count choice is null for some reason (should not be null ever)");
            resetOptions(0, UpdateStages.STARTING_LOCATION);
            return;
        }

        // FIRST SCORE CHOOSER
        // this only returns empty when startingPositionChooser.getSelected() is null so we can use this as null check
        if (cycleNum == 0 && stageCalledFrom == UpdateStages.STARTING_LOCATION) {
            passedDangerPoint = true;
            Optional<ArrayList<ReefLocs>> optimalFirstScoresOptional = getOptimalReefLocs(startingPositionChooser.getSelected());
            if (optimalFirstScoresOptional.isEmpty()) {
                resetOptions(0, UpdateStages.SCORE_LOCATION);
                return;
            }
            ArrayList<ReefLocs> optimalFirstScores = optimalFirstScoresOptional.get();
            firstScoreChooser.close();
            
            firstScoreChooser = new SendableChooser<ReefLocs>();
            for (int i = 0; i < optimalFirstScores.size(); i++) {
                firstScoreChooser.addOption(optimalFirstScores.get(i).name(), optimalFirstScores.get(i));
            }
            firstScoreChooser.setDefaultOption(AutonChooserK.kDefaultNullMessage, null);
            firstScoreChooser.onChange(choice -> updateChoices(cycleCountChooser.getSelected(), 0, UpdateStages.SCORE_LOCATION));
            SmartDashboard.putData(AutonChooserK.kFirstScoreLocChooserKey, firstScoreChooser);
        }

        // FIRST HP CHOOSER
        if ((cycleNum == 0 && stageCalledFrom == UpdateStages.SCORE_LOCATION) || passedDangerPoint) {
            Optional<ArrayList<HPStation>> optimalFirstHPStationsOptional = getOptimalHPStations(firstScoreChooser.getSelected());
            if (optimalFirstHPStationsOptional.isEmpty()) {
                resetOptions(0, UpdateStages.HP_STATION_LOCATION);
                return;
            }
            ArrayList<HPStation> optimalFirstHPStations = optimalFirstHPStationsOptional.get();
            firstHPStationChooser.close();

            firstHPStationChooser = new SendableChooser<HPStation>();
            for (int i = 0; i < optimalFirstHPStations.size(); i++) {
                firstHPStationChooser.addOption(optimalFirstHPStations.get(i).name(), optimalFirstHPStations.get(i));
            }
            firstHPStationChooser.setDefaultOption(AutonChooserK.kDefaultNullMessage, null);
            firstHPStationChooser.onChange(choice -> updateChoices(cycleCountChooser.getSelected(), 0, UpdateStages.HP_STATION_LOCATION));
            SmartDashboard.putData(AutonChooserK.kFirstHPStationChooserKey, firstHPStationChooser);
        }
    }

    /**
     * Uses keys defined through {@link configGeneratedNTKeys()} to generate full keys for NT
     * @param cycle Cycle chooser needs to refer to
     * @param updateStage UpdateStage that chooser refers to
     * @return The key that it should use in network tables
     */
    private static String generateKey(int cycle, UpdateStages updateStage) {
        return cycle + updateStageToKeyText.get(updateStage);
    }

    /**
     * This generates the text required for keys for cycle choosers. Cycle number added in generateKey
     */
    private static void configGeneratedNTKeys() {
        updateStageToKeyText.put(UpdateStages.STARTING_LOCATION, "DO NOT SEE THIS CHOOSER");
        updateStageToKeyText.put(UpdateStages.SCORE_LOCATION, " - SCORE LOCATION");
        updateStageToKeyText.put(UpdateStages.HP_STATION_LOCATION, " - HP STATION");
        updateStageToKeyText.put(UpdateStages.ELE_HEIGHT, " - ELE HEIGHT");
    } 

    private static void printNullOptionChanged() {
        System.out.println("ALERT A CHOOSER WITH NO OPTIONS JUST CHANGED GANG WTF");
    }

    private static enum UpdateStages {
        STARTING_LOCATION(-1, "STARTING_LOCATION"),
        SCORE_LOCATION(0, "SCORE_LOCATION"),
        HP_STATION_LOCATION(1, "HP_STATION_LOCATION"),
        ELE_HEIGHT(2, "ELE_HEIGHT");

        public final int m_index;
        public final String m_name;

        private UpdateStages(int index, String name) {
            m_index = index;
            m_name = name;
        }
    }

    // ---- STANDALONE UTILITIES ----
    // These can likely be made public if needed else where

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
}
