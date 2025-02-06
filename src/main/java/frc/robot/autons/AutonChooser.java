package frc.robot.autons;

import java.util.EnumMap;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autons.TrajsAndLocs.*;

public class AutonChooser {

    private static EnumMap<StartingLocs, String> startingLocMap = new EnumMap<>(TrajsAndLocs.StartingLocs.class);
    private static SendableChooser<StartingLocs> startingPositionChooser = new SendableChooser<StartingLocs>();

    private static EnumMap<ReefLocs, String> firstScoringMap = new EnumMap<>(TrajsAndLocs.ReefLocs.class);
    private static SendableChooser<ReefLocs> firstScoringChooser = new SendableChooser<ReefLocs>();

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

}
