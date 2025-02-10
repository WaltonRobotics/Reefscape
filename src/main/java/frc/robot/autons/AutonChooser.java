package frc.robot.autons;

import java.util.EnumMap;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autons.TrajsAndLocs.*; 

public class AutonChooser {

    private static EnumMap<StartingLocs, String> startingLocMap = new EnumMap<>(TrajsAndLocs.StartingLocs.class);
    private static SendableChooser<StartingLocs> startingPositionChooser = new SendableChooser<StartingLocs>();

    private static EnumMap<ReefLocs, String> firstScoringMap = new EnumMap<>(TrajsAndLocs.ReefLocs.class);
    private static SendableChooser<ReefLocs> firstScoringChooser = new SendableChooser<ReefLocs>();
    
    private static Supplier<StartingLocs> startLocChosen = () -> startingPositionChooser.getSelected(); //need to constant check if val of startloc changed

    private static EnumMap<HPStation, String> hpStationMap = new EnumMap<>(TrajsAndLocs.HPStation.class);
    private static SendableChooser<HPStation> hpStationChooser = new SendableChooser<HPStation>();

    private static Supplier<ReefLocs> reefChosen = () -> firstScoringChooser.getSelected();

    private static EnumMap<ReefLocs, String> hpToReefMap = new EnumMap<>(TrajsAndLocs.ReefLocs.class); 
    private static SendableChooser<ReefLocs> hpToReefChooser = new SendableChooser<ReefLocs>();
    
    private static Supplier<HPStation> hpStationChosen = () -> hpStationChooser.getSelected();

    private static EnumMap<HPStation, String> reefToHPMap = new EnumMap<>(TrajsAndLocs.HPStation.class); 
    private static SendableChooser<HPStation> reefToHPChooser = new SendableChooser<HPStation>();

    static{
        SmartDashboard.putData("starting position chooser", startingPositionChooser);   
        SmartDashboard.putData("human player station chooser", hpStationChooser);
    }

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

    public static void assignHPStation(HPStation hpstation, String description){
        hpStationMap.put(hpstation, description);
        hpStationChooser.addOption(description, hpstation);
    }
    public static void assignReefScoring(ReefLocs reefLocs, String description){
        hpToReefMap.put(reefLocs, description);
        hpToReefChooser.addOption(description, reefLocs);
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
    public static void chooseHPtoReef(){
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
        SmartDashboard.putData("HP to Reef chooser",hpToReefChooser);
        SmartDashboard.updateValues();
    }
    

    public static void chooseReefToHP(){
        reefToHPChooser = new SendableChooser<HPStation>();

        if(reefChosen.get().equals(TrajsAndLocs.ReefLocs.REEF_A)){
            for (int i = 0; i < TrajsAndLocs.Trajectories.ReefToHPTrajs.size(); i++) {
                
            }
        }
    }
    
    
}
