package frc.robot.autons;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutonChooser {
    public static final AutoChooser autoChooser = new AutoChooser();

    public static void addPathsAndCmds(WaltAutonFactory autonFactory) {
        autoChooser.addRoutine("auton", () -> autonFactory.generateAuton());
        autoChooser.addRoutine("leave-only", () -> autonFactory.leaveOnly());
        autoChooser.addRoutine("score-one", () -> autonFactory.scoreOneSlowly());
        autoChooser.select("auton");

        SmartDashboard.putData("AutonChooser", autoChooser);
    }
}