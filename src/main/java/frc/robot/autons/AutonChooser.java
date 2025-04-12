package frc.robot.autons;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutonChooser {
    public static AutoChooser autoChooser = new AutoChooser();

    public static void resetAutoChooser() {
        autoChooser = new AutoChooser();
    }

    public static void addPathsAndCmds(WaltAutonFactory autonFactory, boolean isMid) {
        autoChooser.addRoutine("auton", () -> autonFactory.generateAuton());
        autoChooser.addRoutine("mid-1", () -> autonFactory.midAuton());
        if(isMid) {
            autoChooser.select("mid-1");
        } else {
            autoChooser.select("auton");
        }

        SmartDashboard.putData("AutonChooser", autoChooser);
    }
}