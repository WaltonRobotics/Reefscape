package frc.robot.autons;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutonChooser {
    public static AutoChooser autoChooser = new AutoChooser();

    public static void resetAutoChooser() {
        autoChooser = new AutoChooser();
    }

    public static void addPathsAndCmds(WaltAutonFactory autonFactory, boolean isShort) {
        autoChooser.addRoutine("auton", () -> autonFactory.generateAuton(isShort));
        autoChooser.addRoutine("leave-only", () -> autonFactory.leaveOnly());
        autoChooser.select("auton");

        SmartDashboard.putData("AutonChooser", autoChooser);
    }
}