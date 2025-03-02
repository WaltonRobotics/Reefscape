package frc.robot.autons;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutonChooser {
    public static final AutoChooser autoChooser = new AutoChooser();

    public static void addPathsAndCmds(WaltAutonFactory autonFactory) {
        autoChooser.addRoutine("auton", () -> autonFactory.generateAuton());
        autoChooser.select("auton");

        SmartDashboard.putData("AutonChooser", autoChooser);
    }
}