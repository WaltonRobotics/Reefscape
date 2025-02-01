package frc.robot.autons;

import java.util.EnumMap;

import choreo.Choreo;
import choreo.trajectory.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autons.TrajsAndLocs.HpStation;
import frc.robot.autons.TrajsAndLocs.FirstScoringLocs;
import frc.robot.autons.TrajsAndLocs.ReefLocation;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.EleHeight;

public class AutonChooser {

    public enum StartAuton{
        REEF_H("Reef H Start", TrajsAndLocs.FirstScoringLocs.REEF_H),
        REEF_I("Reef I Start", TrajsAndLocs.FirstScoringLocs.REEF_I),
        REEF_J("Reef J Start", TrajsAndLocs.FirstScoringLocs.REEF_J),

        REEF_E("Reef E Start", TrajsAndLocs.FirstScoringLocs.REEF_E),
        REEF_F("Reef F Start", TrajsAndLocs.FirstScoringLocs.REEF_F),
        REEF_G("Reef G Start", TrajsAndLocs.FirstScoringLocs.REEF_G);


        public final String m_description;
        public final FirstScoringLocs m_traj;

        StartAuton(String description, FirstScoringLocs traj){
            m_description = description;
            m_traj = traj;
        }

    }

    public enum EleAutonHeights {
        HOME("Home", EleHeight.HOME),
        L1("L1", EleHeight.L1),
        L2("L2", EleHeight.L2),
        L3("L3", EleHeight.L3),
        L4("L4", EleHeight.L4);

        public final String m_description;
        public final EleHeight m_heightMeters;

       private EleAutonHeights(String description, EleHeight eleHeights){
                   m_description = description;
                   m_heightMeters = eleHeights;
        }
    }

    public enum CSOptions {
        LEFT("Left", HpStation.CS_LEFT),
        RIGHT("Right", HpStation.CS_RIGHT);

        public final String m_description;
        public final HpStation m_coralStation;

        private CSOptions(String description, HpStation coralStation){
            m_description = description;
            m_coralStation = coralStation;
        }
    }

    public enum ScoringAuton{
        REEF_A("Reef A End", TrajsAndLocs.ReefLocation.REEF_A),
        REEF_B("Reef B End", TrajsAndLocs.ReefLocation.REEF_B),
        REEF_C("Reef C End", TrajsAndLocs.ReefLocation.REEF_C),
        REEF_D("Reef D End", TrajsAndLocs.ReefLocation.REEF_D),
        REEF_E("Reef E End", TrajsAndLocs.ReefLocation.REEF_E),
        REEF_F("Reef F End", TrajsAndLocs.ReefLocation.REEF_F),
        REEF_G("Reef G End", TrajsAndLocs.ReefLocation.REEF_G),
        REEF_H("Reef H End", TrajsAndLocs.ReefLocation.REEF_H),
        REEF_I("Reef I End", TrajsAndLocs.ReefLocation.REEF_I),
        REEF_J("Reef J End", TrajsAndLocs.ReefLocation.REEF_J),
        REEF_K("Reef K End", TrajsAndLocs.ReefLocation.REEF_K),
        REEF_L("Reef L End", TrajsAndLocs.ReefLocation.REEF_L);
        

        public final String m_description;
        public final ReefLocation m_traj;

        ScoringAuton(String description, ReefLocation traj){
            m_description = description;
            m_traj = traj;
        }
    }

    private AutonChooser(){

    }

    private static EnumMap<StartAuton, FirstScoringLocs> startChooserMap = new EnumMap<>(StartAuton.class);
    private static SendableChooser<StartAuton> startAutonNTChooser = new SendableChooser<StartAuton>();

    private static EnumMap<ScoringAuton, ReefLocation> scoringChooserMap = new EnumMap<>(ScoringAuton.class);
    private static SendableChooser<ScoringAuton> scoringChooser = new SendableChooser<ScoringAuton>();

    private static EnumMap<EleAutonHeights, EleHeight> eleChooserMap = new EnumMap<>(EleAutonHeights.class);
    private static SendableChooser<EleAutonHeights> eleAutonNTChooser = new SendableChooser<EleAutonHeights>();

    private static EnumMap<CSOptions, HpStation> coralChooserMap = new EnumMap<>(CSOptions.class);
    private static SendableChooser<CSOptions> coralAutonNTChooser = new SendableChooser<CSOptions>();


    static{
        SmartDashboard.putData("StartAuton", startAutonNTChooser);
        SmartDashboard.putData("EleChooser", eleAutonNTChooser);
        SmartDashboard.putData("CoralChooser", coralAutonNTChooser);
        SmartDashboard.putData("ScoringAuton",scoringChooser);
    }

    public static void assignAutonCommand(StartAuton auton, FirstScoringLocs firstScoringLocs){
        startChooserMap.put(auton, firstScoringLocs);
        startAutonNTChooser.addOption(auton.m_description, auton);
        startAutonNTChooser.onChange(null);
    }

    public static void assignAutonCommand(EleAutonHeights eleChooser, EleHeight height){
        eleChooserMap.put(eleChooser, height);
        eleAutonNTChooser.addOption(eleChooser.m_description, eleChooser);
    }

    public static void assignAutonCommand(CSOptions coralChooser, HpStation coralStation){
        coralChooserMap.put(coralChooser, coralStation);
        coralAutonNTChooser.addOption(coralChooser.m_description, coralChooser);
    }
    public static void assignAutonCommand(ScoringAuton auton, ReefLocation scoringLocs){
        scoringChooserMap.put(auton, scoringLocs);
        scoringChooser.addOption(auton.m_description, auton);
    }

}
