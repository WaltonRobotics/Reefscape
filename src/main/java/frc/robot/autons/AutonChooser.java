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

    private static Supplier<StartingLocs> wheeeee = () -> startingPositionChooser.getSelected();



    static{
        SmartDashboard.putData("starting position chooser", startingPositionChooser);
        SmartDashboard.putData("first scoring chooser", firstScoringChooser);
    }

    public static void assignPosition(StartingLocs startingLoc, String description){
        startingLocMap.put(startingLoc, description);
        startingPositionChooser.addOption(description, startingLoc);
    }

    public static void assignFirstScoring(ReefLocs scoringLoc, String description){
        firstScoringMap.put(scoringLoc, description);
        firstScoringChooser.addOption(description, scoringLoc);
    }
    public static void setDefaultAuton(StartingLocs scoringLoc, String description){
        startingPositionChooser.setDefaultOption("hahatest", scoringLoc);
    }

    // public void periodic() {
    //     if(wheeeee.get().equals(TrajsAndLocs.StartingLocs.MID)){
    //         for (int i = 0; i < TrajsAndLocs.ReefLocs.OptimalLeftStartCycles.size(); i++) {
    //             assignFirstScoring(TrajsAndLocs.ReefLocs.OptimalLeftStartCycles.get(i), TrajsAndLocs.ReefLocs.OptimalLeftStartCycles.get(i).toString());
    //         }
            
    //     }else {
    //         for (int i = 0; i < TrajsAndLocs.ReefLocs.OptimalRightStartCycles.size(); i++) {
    //             assignFirstScoring(TrajsAndLocs.ReefLocs.OptimalRightStartCycles.get(i), TrajsAndLocs.ReefLocs.OptimalRightStartCycles.get(i).toString());
    //         }
    //     }
    // }

    public static void chooseFirstScoring(){
        if(wheeeee.get().equals(TrajsAndLocs.StartingLocs.MID)){
            // for (int i = 0; i < TrajsAndLocs.ReefLocs.OptimalLeftStartCycles.size(); i++) {
            //     assignFirstScoring(TrajsAndLocs.ReefLocs.OptimalLeftStartCycles.get(i), TrajsAndLocs.ReefLocs.OptimalLeftStartCycles.get(i).toString());
            // }
            assignFirstScoring(TrajsAndLocs.ReefLocs.REEF_C, "reef c");
        }else {
            // for (int i = 0; i < TrajsAndLocs.ReefLocs.OptimalRightStartCycles.size(); i++) {
            //     assignFirstScoring(TrajsAndLocs.ReefLocs.OptimalRightStartCycles.get(i), TrajsAndLocs.ReefLocs.OptimalRightStartCycles.get(i).toString());
            // }
            assignFirstScoring(TrajsAndLocs.ReefLocs.REEF_B, "reef b"); 
        }

        
    }

}
