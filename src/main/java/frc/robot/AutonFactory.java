package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class AutonFactory {
    private final AutoFactory m_autoFactory;
    private Map<Pair<Location, Location>, String> m_trajectoryMap = new HashMap<Pair<Location, Location>, String>();

    public AutonFactory(AutoFactory autoFactory) {
        m_autoFactory = autoFactory;
        configureTrajectories();
    }

    public Optional<AutoRoutine> generateAutoRoutine(List<Location> locations, List<Command> parallelActions, List<Command> actions) {
        // list lengths have to match up exactly AND you must have at least 2 locations (locations should have one more element than the others)
        if (locations.size() < 2) {
            System.out.println("AUTON GENERATION FAILURE: Fewer than 2 locations");
            return Optional.empty();
        }
        if (locations.size() != parallelActions.size() + 1) {
            System.out.println("AUTON GENERATION FAILURE: Number of parallel actions + 1 not equal to number of locations");
            return Optional.empty();
        }
        if (locations.size() != actions.size() + 1) {
            System.out.println("AUTON GENERATION FAILURE: Number of actions + 1 not equal to number of locations");
            return Optional.empty();
        }

        final AutoRoutine routine = m_autoFactory.newRoutine("auton");

        List<AutoTrajectory> trajectories = new ArrayList<AutoTrajectory>();
        for (int i = 0; i < locations.size() - 1; i++) {
            Pair<Location, Location> currentKey = new Pair<>(locations.get(i), locations.get(i+1));
            // if there is corresponding trajectory for an entered location pair, stop the auton
            // if this wasn't checked we'd pass null to a different place
            if (m_trajectoryMap.get(currentKey) == null) {
                System.out.println("AUTON GENERATION FAILURE: Trajectory not found for location pair starting at index " + i);
                return Optional.empty();
            }

            trajectories.add(
                routine.trajectory(m_trajectoryMap.get(
                    currentKey
                ))
            );
        }

        routine.active().onTrue(
            Commands.sequence(
                m_autoFactory.resetOdometry("Trajectory 1"),
                Commands.parallel(
                    trajectories.get(0).cmd(),
                    parallelActions.get(0)
                )
            )
        );

        Trigger firstPortionDone = trajectories.get(0).done().and(new Trigger(parallelActions.get(0)::isFinished));
        routine.active().and(firstPortionDone.onTrue(actions.get(0)));

        for (int i = 1; i < trajectories.size(); i++) {
            // WAIT FOR ACTION TO FINISH
            Trigger previousActionDone = new Trigger(actions.get(i-1)::isFinished);

            // PERFORM OWN TRAJ + PARALLEL ACTION
            routine.active().and(previousActionDone).onTrue(
                Commands.parallel(
                    trajectories.get(i).cmd(),
                    parallelActions.get(i)
                )
            );

            // WAIT FOR TRAJ + PARALLEL ACTION TO FINISH
            Trigger portionDone = trajectories.get(i).done().and(new Trigger(parallelActions.get(i)::isFinished));

            // PERFORM OWN ACTION
            routine.active().and(portionDone).onTrue(actions.get(i));
        }

        return Optional.of(routine);
    }

    public void configureTrajectories() {
        m_trajectoryMap.put(new Pair<Location, Location>(Location.STARTING_LOCATION_A, Location.REEF_A), "STARTING_LOCATION_A -> REEF_A");
        m_trajectoryMap.put(new Pair<Location, Location>(Location.REEF_A, Location.STATION_A), "REEF_A -> STATION_A");
    }

    public enum Location {
        STARTING_LOCATION_A,
        STARTING_LOCATION_B,
        REEF_A,
        REEF_B,
        REEF_C,
        STATION_A,
        STATION_B
    }
}
