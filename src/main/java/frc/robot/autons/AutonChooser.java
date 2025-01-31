package frc.robot.autons;

import java.util.EnumMap;

import choreo.Choreo;
import choreo.trajectory.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autons.TrajsAndLocs.FirstScoringLocs;
import frc.robot.autons.TrajsAndLocs.ScoringLocs;

public class AutonChooser {

    public enum AutonOption{
        MEOW("Reef 1", TrajsAndLocs.FirstScoringLocs.REEF_1),
        GROWL("Reef 2", TrajsAndLocs.FirstScoringLocs.REEF_2),
        BARK("Reef 3", TrajsAndLocs.FirstScoringLocs.REEF_3);

        public final String m_description;
        public final FirstScoringLocs m_traj;

        AutonOption(String description, FirstScoringLocs traj){
            m_description = description;
            m_traj = traj;
        }

    }


    private AutonChooser(){

    }

    private static EnumMap<AutonOption, FirstScoringLocs> autonChooserMap = new EnumMap<>(AutonOption.class);
    private static SendableChooser<AutonOption> autonNTChooser = new SendableChooser<AutonOption>();

    static{
        SmartDashboard.putData("AutonChooser", autonNTChooser);
    }

    public static void assignAutonCommand(AutonOption auton, FirstScoringLocs firstScoringLocs){
        autonChooserMap.put(auton, firstScoringLocs);
        autonNTChooser.addOption(auton.m_description, auton);
    }

}
