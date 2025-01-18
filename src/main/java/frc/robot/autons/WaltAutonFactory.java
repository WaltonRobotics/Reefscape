package frc.robot.autons;

import java.util.ArrayList;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;

/* 
 * see https://drive.google.com/file/d/1dWnc3RpS5WJw0GfrmbV2ZbxEjuDWTSV-/view?usp=drive_link for auton naming conventions
 * diagram is mirrored for each side
 */

public class WaltAutonFactory {
    private final AutoFactory m_factory;
    public static AutoRoutine m_routine;

    public WaltAutonFactory(AutoFactory factory) {
        m_factory = factory;
        m_routine = m_factory.newRoutine("auton"); 
    }

    public enum Paths {
        MID_A_1("scores on 1 from starting pos and goes to csA", m_routine.trajectory("mid_a_1")),
        MID_B_12("scores on 2 from starting pos and goes to csB", m_routine.trajectory("mid_a_2")),

        CS_A_1("scores on 1 from coral station A", m_routine.trajectory("cs_a_1")),
        CS_A_2("scores on 2 from coral station A", m_routine.trajectory("cs_a_2")),
        CS_A_3("scores on 3 from coral station A", m_routine.trajectory("cs_a_3")),
        CS_A_4("scores on 4 from coral station A", m_routine.trajectory("cs_a_4")),
        CS_A_5("scores on 5 from coral station A", m_routine.trajectory("cs_a_5")),
        CS_A_6("scores on 6 from coral station A", m_routine.trajectory("cs_a_6")),
        // A 7-10 left out cuz why would u ever need that
        CS_A_11("scores on 11 from coral station A", m_routine.trajectory("cs_a_11")),
        CS_A_12("scores on 12 from coral station A", m_routine.trajectory("cs_a_12")),
    
        CS_B_1("scores on 1 from coral station B", m_routine.trajectory("cs_b_1")),
        CS_B_2("scores on 2 from coral station B", m_routine.trajectory("cs_b_2")),
        // B 3-6 left out cuz why would u ever need that
        CS_B_7("scores on 7 from coral station B", m_routine.trajectory("cs_b_7")),
        CS_B_8("scores on 8 from coral station B", m_routine.trajectory("cs_b_8")),
        CS_B_9("scores on 9 from coral station B", m_routine.trajectory("cs_b_9")),
        CS_B_10("scores on 10 from coral station B", m_routine.trajectory("cs_b_10")),
        CS_B_11("scores on 11 from coral station B", m_routine.trajectory("cs_b_11")),
        CS_B_12("scores on 12 from coral station B", m_routine.trajectory("cs_b_12"));

        public final String m_description;
        public final ArrayList<AutoTrajectory> m_trajs;
        
        Paths(String description, AutoTrajectory... trajs) {
            m_description = description;
            m_trajs = new ArrayList<AutoTrajectory>();
            for (AutoTrajectory traj : trajs) {
                m_trajs.add(traj);
            }
        }
    }

    /*
     * rip ezTest1. u were a good first testing auto :( we will (NAWT) miss you
    */
    // public AutoRoutine ezTest1() {
    //     final AutoRoutine routine = m_factory.newRoutine("ezTest1");

    //     //trajs
    //     AutoTrajectory ezTest1Traj = routine.trajectory("ez_test_1");

    //     routine.active().onTrue(
    //         Commands.sequence(
    //             ezTest1Traj.resetOdometry(),
    //             ezTest1Traj.cmd()
    //         )
    //     );

    //     return routine;
    // }

    /*
     * BASE AUTONS
     * covers the starting position -> first coral scored -> going to a coral station
     */
    

    /*
     * AUTON APPENDER
     * takes in a path, appends to the base auton
     */

}
