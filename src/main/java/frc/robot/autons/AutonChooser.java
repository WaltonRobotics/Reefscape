package frc.robot.autons;

import java.io.ObjectOutputStream.PutField;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autons.TrajsAndLocs.*; 

public class AutonChooser {
    public static enum NumCycles {
        CYCLE_1(1),
        CYCLE_2(2),
        CYCLE_3(3),
        CYCLE_4(4);

        public int m_cycles;

        private NumCycles(int cycles){
            m_cycles = cycles;
        }
        
    }

    private static EnumMap<StartingLocs, String> startingLocMap = new EnumMap<>(TrajsAndLocs.StartingLocs.class);
    private static SendableChooser<StartingLocs> startingPositionChooser = new SendableChooser<StartingLocs>();

    private static Supplier<StartingLocs> startLocChosen = () -> startingPositionChooser.getSelected();

    private static EnumMap<ReefLocs, String> firstScoringMap = new EnumMap<>(TrajsAndLocs.ReefLocs.class);
    private static SendableChooser<ReefLocs> firstScoringChooser = new SendableChooser<ReefLocs>();
    
    private static Supplier<ReefLocs> firstScoringChosen = () -> firstScoringChooser.getSelected();
   
    private static EnumMap<HPStation, String> hpStationMap = new EnumMap<>(TrajsAndLocs.HPStation.class);
    private static SendableChooser<HPStation> hpStationChooser = new SendableChooser<HPStation>();

    private static Supplier<HPStation> hpStationChosen = () -> hpStationChooser.getSelected();

    private static EnumMap<ReefLocs, String> hpToReefMap = new EnumMap<>(TrajsAndLocs.ReefLocs.class); 
    private static SendableChooser<ReefLocs> hpToReefChooser = new SendableChooser<ReefLocs>();
    
    private static Supplier<ReefLocs> reefChosen = () -> hpToReefChooser.getSelected();

    private static EnumMap<HPStation, String> reefToHPMap = new EnumMap<>(TrajsAndLocs.HPStation.class); 
    private static SendableChooser<HPStation> reefToHPChooser = new SendableChooser<HPStation>();

    private static Supplier<HPStation> reefToHPChosen = () -> reefToHPChooser.getSelected();

    private static EnumMap<NumCycles, String> cyclesMap = new EnumMap<>(NumCycles.class);
    private static SendableChooser<NumCycles> cyclesChooser = new SendableChooser<NumCycles>();

    private static Supplier<NumCycles> cyclesChosen = () -> cyclesChooser.getSelected();

    static{
        SmartDashboard.putData("starting position chooser", startingPositionChooser);   
        SmartDashboard.putData("human player station chooser", hpStationChooser);
        SmartDashboard.putData("number of cycles chooser", cyclesChooser);
    }

    public static void assignNumCycles(NumCycles numCycles, String description){
        cyclesMap.put(numCycles, description);
        cyclesChooser.addOption(description, numCycles);
    }

    // public static void setDefaultCycles(NumCycles numCycles){
    //     cyclesChooser.setDefaultOption("default (1)", numCycles);
    // }

    public static void assignPosition(StartingLocs startingLoc, String description){
        startingLocMap.put(startingLoc, description);
        startingPositionChooser.addOption(description, startingLoc);
    }

    public static void assignFirstScoring(ReefLocs scoringLoc, String description){
        firstScoringMap.put(scoringLoc, description);
        firstScoringChooser.addOption(description, scoringLoc);
    }

    public static void setDefaultAuton(StartingLocs scoringLoc){
        startingPositionChooser.setDefaultOption("default (mid)", scoringLoc);
    }

    public static void setDefaultHPStation(HPStation hpStation){
        hpStationChooser.setDefaultOption("default(left)", hpStation);
    }

    // public static void setDefaultReefScoring(ReefLocs reefLocs){
    //     hpToReefChooser.setDefaultOption("default(A)", reefLocs);
    // }

    public static void assignHPStation(HPStation hpstation, String description){
        hpStationMap.put(hpstation, description);
        hpStationChooser.addOption(description, hpstation);
    }
    public static void assignReefScoring(ReefLocs reefLocs, String description){
        hpToReefMap.put(reefLocs, description);
        hpToReefChooser.addOption(description, reefLocs);
    }

    public static void assignReeftoHPScoring(HPStation hpStation, String description){
        reefToHPMap.put(hpStation, description);
        reefToHPChooser.addOption(description, hpStation);
    }

    /**
     * depending on certain starting location, displays the optimal path for said starting + scoring path
     */
    public static void chooseFirstScoring(){
        firstScoringChooser = new SendableChooser<ReefLocs>();

        if(startLocChosen.get().equals(TrajsAndLocs.StartingLocs.MID)){

            for (int i = 0; i < TrajsAndLocs.ReefLocs.OptimalMidStartCycles.size(); i++) {
                assignFirstScoring(TrajsAndLocs.ReefLocs.OptimalMidStartCycles.get(i), 
                    TrajsAndLocs.ReefLocs.OptimalMidStartCycles.get(i).toString());
            }

        }else if (startLocChosen.get().equals(TrajsAndLocs.StartingLocs.LEFT)){

            for (int i = 0; i < TrajsAndLocs.ReefLocs.OptimalLeftStartCycles.size(); i++) {
                assignFirstScoring(TrajsAndLocs.ReefLocs.OptimalLeftStartCycles.get(i), 
                    TrajsAndLocs.ReefLocs.OptimalLeftStartCycles.get(i).toString());
            }

        } else{
            
            for (int i = 0; i < TrajsAndLocs.ReefLocs.OptimalRightStartCycles.size(); i++) {
                assignFirstScoring(TrajsAndLocs.ReefLocs.OptimalRightStartCycles.get(i), 
                    TrajsAndLocs.ReefLocs.OptimalRightStartCycles.get(i).toString());
            }

        }   
        SmartDashboard.putData("first scoring chooser", firstScoringChooser);
        SmartDashboard.updateValues(); //FINALLY

    }

    /**
     * given that an HP Station is selected, creates NT that shows all possible(optimal?) routes to certain reefs
     */
    public static void chooseHPtoReef(String description){
        hpToReefChooser = new SendableChooser<ReefLocs>();
        

        if(hpStationChosen.get().equals(TrajsAndLocs.HPStation.HP_RIGHT)){ 
            for(int i = 0; i < TrajsAndLocs.Trajectories.HPToReefTrajs.size() / 2; i++){
                assignReefScoring(TrajsAndLocs.ReefLocs.OptimalRightHPCycles.get(i), 
                TrajsAndLocs.ReefLocs.OptimalRightHPCycles.get(i).toString());
            }
        } else {
            for(int i = 0; i < TrajsAndLocs.Trajectories.HPToReefTrajs.size() / 2; i++){
                assignReefScoring(TrajsAndLocs.ReefLocs.OptimalLeftHPCycles.get(i), 
                TrajsAndLocs.ReefLocs.OptimalLeftHPCycles.get(i).toString());
            }
        }
        SmartDashboard.putData(description, hpToReefChooser);
        SmartDashboard.updateValues();
    }
    
/**
 * given that a reef was selected (after going to HP), creates the possible HP options for that selected reef
 */
    public static void chooseReefToHP(String description){
        reefToHPChooser = new SendableChooser<HPStation>();
        
        if(reefChosen.get() != null){
            if(reefChosen.get().equals(TrajsAndLocs.ReefLocs.REEF_A)){
                assignReeftoHPScoring(TrajsAndLocs.HPStation.HP_LEFT, "hp left");
                assignReeftoHPScoring(TrajsAndLocs.HPStation.HP_RIGHT, "hp right");

            } else if(reefChosen.get().equals(TrajsAndLocs.ReefLocs.REEF_B)){
                assignReeftoHPScoring(TrajsAndLocs.HPStation.HP_LEFT, "hp left");
                assignReeftoHPScoring(TrajsAndLocs.HPStation.HP_RIGHT, "hp right");

            } else if(reefChosen.get().equals(TrajsAndLocs.ReefLocs.REEF_C)){
                assignReeftoHPScoring(TrajsAndLocs.HPStation.HP_RIGHT, "hp right");

            } else if(reefChosen.get().equals(TrajsAndLocs.ReefLocs.REEF_D)){
                assignReeftoHPScoring(TrajsAndLocs.HPStation.HP_RIGHT, "hp right"); 

            } else if(reefChosen.get().equals(TrajsAndLocs.ReefLocs.REEF_E)){
                assignReeftoHPScoring(TrajsAndLocs.HPStation.HP_RIGHT, "hp right");

            } else if(reefChosen.get().equals(TrajsAndLocs.ReefLocs.REEF_F)){
                assignReeftoHPScoring(TrajsAndLocs.HPStation.HP_RIGHT, "hp right");

            } else if(reefChosen.get().equals(TrajsAndLocs.ReefLocs.REEF_G)){
                assignReeftoHPScoring(TrajsAndLocs.HPStation.HP_LEFT, "hp left");
                assignReeftoHPScoring(TrajsAndLocs.HPStation.HP_RIGHT, "hp right");

            } else if(reefChosen.get().equals(TrajsAndLocs.ReefLocs.REEF_H)){
                assignReeftoHPScoring(TrajsAndLocs.HPStation.HP_LEFT, "hp left");
                assignReeftoHPScoring(TrajsAndLocs.HPStation.HP_RIGHT, "hp right");

            } else if(reefChosen.get().equals(TrajsAndLocs.ReefLocs.REEF_I)){
                assignReeftoHPScoring(TrajsAndLocs.HPStation.HP_LEFT, "hp left");

            } else if(reefChosen.get().equals(TrajsAndLocs.ReefLocs.REEF_J)){
                assignReeftoHPScoring(TrajsAndLocs.HPStation.HP_LEFT, "hp left");

            } else if(reefChosen.get().equals(TrajsAndLocs.ReefLocs.REEF_K)){
                assignReeftoHPScoring(TrajsAndLocs.HPStation.HP_LEFT, "hp left");

            } else{
                assignReeftoHPScoring(TrajsAndLocs.HPStation.HP_LEFT, "hp left");

            }
        }

        SmartDashboard.putData(description, reefToHPChooser);
        SmartDashboard.updateValues();
    }

    public static void cycleIterations(){

        int amount = 1;

        if(cyclesChosen.get() != null){

            if(cyclesChosen.get().equals(NumCycles.CYCLE_2)){
                amount = 2;
            } else if(cyclesChosen.get().equals(NumCycles.CYCLE_3)){
                amount = 3;
            } else if(cyclesChosen.get().equals(NumCycles.CYCLE_4)){
                amount = 4;
            }  

            if(hpStationChosen != null && startLocChosen != null && firstScoringChosen != null){
                startingPositionChooser.close();
                firstScoringChooser.close();
                hpStationChooser.close();
            }

            for(int i = 1; i <= amount; i++){
                chooseHPtoReef("HP to Reef Chooser " + i);
                chooseReefToHP("Reef to HP Chooser " + i);
                chooseReefToHP("HP to Reef Chooser");

                ;
            }
        }
    }
}