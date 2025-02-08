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
    
    private static Supplier<StartingLocs> startLocChosen = () -> startingPositionChooser.getSelected();

    private static EnumMap<HPStation, String> hpStationMap = new EnumMap<>(TrajsAndLocs.HPStation.class);
    private static SendableChooser<HPStation> hpStationChooser = new SendableChooser<HPStation>();

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

    public static void chooseHPStation(HPStation hpstation, String description){
        hpStationMap.put(hpstation, description);
        hpStationChooser.addOption(description, hpstation);
    }

    public static void chooseFirstScoring(){
        if(startLocChosen.get().equals(TrajsAndLocs.StartingLocs.MID)){
            
            firstScoringChooser = new SendableChooser<ReefLocs>();
            SmartDashboard.putData("first scoring chooser", firstScoringChooser);

            for (int i = 0; i < TrajsAndLocs.ReefLocs.OptimalMidStartCycles.size(); i++) {
                assignFirstScoring(TrajsAndLocs.ReefLocs.OptimalMidStartCycles.get(i), 
                    TrajsAndLocs.ReefLocs.OptimalMidStartCycles.get(i).toString());
            }

        }else if (startLocChosen.get().equals(TrajsAndLocs.StartingLocs.LEFT)){

            firstScoringChooser = new SendableChooser<ReefLocs>();
            SmartDashboard.putData("first scoring chooser", firstScoringChooser);

            for (int i = 0; i < TrajsAndLocs.ReefLocs.OptimalLeftStartCycles.size(); i++) {
                assignFirstScoring(TrajsAndLocs.ReefLocs.OptimalLeftStartCycles.get(i), 
                    TrajsAndLocs.ReefLocs.OptimalLeftStartCycles.get(i).toString());
            }
        } else{

            firstScoringChooser = new SendableChooser<ReefLocs>();
            SmartDashboard.putData("first scoring chooser", firstScoringChooser);

            for (int i = 0; i < TrajsAndLocs.ReefLocs.OptimalRightStartCycles.size(); i++) {
                assignFirstScoring(TrajsAndLocs.ReefLocs.OptimalRightStartCycles.get(i), 
                    TrajsAndLocs.ReefLocs.OptimalRightStartCycles.get(i).toString());
            }
        }   
    }
    
}
