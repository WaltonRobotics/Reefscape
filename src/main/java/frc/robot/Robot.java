// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.Optional;
import java.util.Set;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.FieldK;
import frc.robot.Constants.VisionK;
import frc.robot.autons.AutonChooser;
import frc.robot.autons.WaltAutonBuilder;
import frc.robot.autons.TrajsAndLocs.HPStation;
import frc.robot.autons.TrajsAndLocs.ReefLocs;
import frc.robot.autons.TrajsAndLocs.StartingLocs;
import frc.robot.autons.WaltAutonBuilder.NumCycles;

import static frc.robot.autons.TrajsAndLocs.ReefLocs.*;

import frc.robot.autons.WaltAutonFactory;
import frc.robot.generated.TunerConstants;
import frc.util.AllianceFlipUtil;
import frc.robot.subsystems.Elevator.AlgaeHeight;
import frc.robot.subsystems.Elevator.EleHeight;
import frc.robot.vision.Vision;
import frc.robot.vision.VisionSim;
import frc.robot.subsystems.*;

public class Robot extends TimedRobot {

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  private final Telemetry logger = new Telemetry(MaxSpeed);

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController manipulator = new CommandXboxController(1);

  public final Swerve drivetrain = TunerConstants.createDrivetrain();
  private final Coral coral = new Coral();
  private final Finger finger = new Finger();
  private final Elevator elevator = new Elevator();
  private final Algae algae;
  private final Superstructure superstructure;

  private Command m_autonomousCommand;
  // VisionSim could probably be static or a singleton instead of this reference mess but that's extra work to potentially break something
  private final VisionSim visionSim = new VisionSim();
  private final Vision eleForwardsCam = new Vision(VisionK.kElevatorForwardsCamName, VisionK.kElevatorForwardsCamSimVisualName,
    VisionK.kElevatorForwardsCamRoboToCam, visionSim, VisionK.kEleForwardCamSimProps);
  // private final Vision lowerRightCam = new Vision(VisionK.kLowerRightCamName, VisionK.kLowerRightCamSimVisualName,
  //   VisionK.kLowerRightCamRoboToCam, visionSim, VisionK.kLowerRightCamSimProps);

  // this should be updated with all of our cameras
  private final Vision[] cameras = {eleForwardsCam};  // lower right cam removed

  private final AutoFactory autoFactory = drivetrain.createAutoFactory();
  private WaltAutonFactory waltAutonFactory = null;

 private final Trigger trg_leftTeleopAutoAlign = driver.x();
  private final Trigger trg_rightTeleopAutoAlign = driver.a();

  // private final Trigger trg_teleopEleHeightReq;
  // sameer wanted b to be his ele override button also, so i created a trigger to check that he didnt mean to press any other override when using b
  // private final Trigger trg_eleOverride;
  private ArrayList<ReefLocs> scoreLocs = new ArrayList<>(List.of(REEF_E, REEF_D, REEF_C)); // dummies
  private ArrayList<EleHeight> heights = new ArrayList<>(List.of(EleHeight.L4, EleHeight.L4, EleHeight.L4));
  private ArrayList<HPStation> hpStations = new ArrayList<>(List.of(HPStation.HP_RIGHT, HPStation.HP_RIGHT, HPStation.HP_RIGHT));

  private final Trigger trg_intakeReq = manipulator.rightBumper();
  
  private final Trigger trg_toL1 = manipulator.povDown();
  private final Trigger trg_toL2 = manipulator.povRight();
  private final Trigger trg_toL3 = manipulator.povLeft();
  private final Trigger trg_toL4 = manipulator.povUp();

  private final Trigger trg_teleopScoreReq = driver.rightTrigger(); 

  private final Trigger trg_algaeIntake = manipulator.a();
  private final Trigger trg_processorReq = manipulator.y();
  private final Trigger trg_shootReq = manipulator.rightTrigger();
  private final Trigger trg_deAlgae = manipulator.leftTrigger();

  // simulation
  private final Trigger trg_simBotBeamBreak = manipulator.leftStick();
  private final Trigger trg_simTopBeamBreak = manipulator.rightStick();
 

  // override button
  private final Trigger trg_driverDanger = driver.b();
  private final Trigger trg_manipDanger = manipulator.b();
  private final Trigger trg_inOverride = trg_manipDanger.or(trg_driverDanger);

  private final SwerveRequest straightWheelsReq = new SwerveRequest.PointWheelsAt().withModuleDirection(new Rotation2d());

  private final StructPublisher<Pose2d> log_robotPose = NetworkTableInstance.getDefault()
        .getStructTopic("Robot/Pose", Pose2d.struct).publish();

  /* WaltAutonBuilder vars */
  private boolean numCycleChange = false;
  private boolean startingPositionChange = false;
  private boolean firstScoringPositionChange = false;
  private boolean startingHeightChange = false;
  private boolean initialHPStationChange = false;

  private boolean beforeAuton = true;
  private boolean autonNotMade = true;
  private boolean readyToMakeAuton = false;

  /* WaltAutonBuilder trigs */
  // When the user selects a different option, this thing runs
  private final Consumer<NumCycles> cyclesConsumer = numCycles -> {
    WaltAutonBuilder.m_cycles = numCycles;
    numCycleChange = true;
  };

  private final Consumer<StartingLocs> startingPositionConsumer = startingPosition -> {
    WaltAutonBuilder.startingPosition = startingPosition;
    startingPositionChange = true;
  };

  private final Consumer<EleHeight> startingHeightConsumer = startingHeight -> {
    WaltAutonBuilder.startingHeight = startingHeight;
    startingHeightChange = true;
  };

  private final Consumer<ReefLocs> initialScoringPositionConsumer = scoringPosition -> {
    WaltAutonBuilder.scoringPosition = scoringPosition;
    firstScoringPositionChange = true;
  };

  private final Consumer<HPStation> initialHPStationConsumer = hpStation -> {
    WaltAutonBuilder.hpStation = hpStation;
    initialHPStationChange = true;
  };

  private final Field2d robotField = visionSim.getSimDebugField();

  public Robot() {
    DriverStation.silenceJoystickConnectionWarning(true);
    if (Robot.isReal()) {
      superstructure = new Superstructure(
      coral,
      finger,
      elevator, 
      trg_intakeReq,
      trg_toL1,
      trg_toL2,
      trg_toL3,
      trg_toL4,
      trg_teleopScoreReq,
      trg_deAlgae.and(trg_toL2),
      trg_deAlgae.and(trg_toL3),
      trg_inOverride,
      new Trigger(() -> false),
      new Trigger(() -> false),
      this::driverRumble);
    } else {
      superstructure = new Superstructure(
      coral,
      finger,
      elevator, 
      trg_intakeReq,
      trg_toL1,
      trg_toL2,
      trg_toL3,
      trg_toL4,
      trg_teleopScoreReq,
      trg_deAlgae.and(trg_toL2),
      trg_deAlgae.and(trg_toL3),
      trg_inOverride,
      trg_simTopBeamBreak,
      trg_simBotBeamBreak,
      this::driverRumble);
    }
      
      algae = new Algae(
        trg_algaeIntake, 
        new Trigger(() -> false), 
        trg_shootReq, 
        this::manipRumble
      );

    waltAutonFactory = new WaltAutonFactory(
      elevator,
      autoFactory, 
      superstructure, 
      drivetrain,
      StartingLocs.RIGHT, 
      scoreLocs,
      heights, 
      hpStations,
      false);

    AutonChooser.addPathsAndCmds(waltAutonFactory);

    configureBindings();
    // configureTestBindings();
  }

  private void configureTestBindings() {

    drivetrain.setDefaultCommand(
          // Drivetrain will execute this command periodically
          drivetrain.applyRequest(() ->
              drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with Y (forward)
                  .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with X (left)
                  .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
          )
      );
    // driver.a().onTrue(
    //   Commands.sequence(
    //     algae.toAngle(WristPos.GROUND),
    //     algae.intake()
    //   )
    // ).onFalse(algae.toAngle(WristPos.HOME));
  
    // driver.y().whileTrue(elevator.testVoltageControl(() -> manipulator.getLeftY()));
    // driver.x().whileTrue(coral.testFingerVoltageControl(() -> manipulator.getLeftY()));

    // driver.x().onTrue(elevator.toHeight(Feet.of(1).in(Meters)));
    // driver.y().onTrue(elevator.toHeight(Inches.of(1).in(Meters)));

    /* 
       * programmer buttons
       * make sure u comment out when not in use
       */
      // Run SysId routines when holding back/start and X/Y.
      // Note that each routine should be run exactly once in a single log.
      driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
      driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
      driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
      driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

      driver.back().and(driver.a()).whileTrue(elevator.sysIdDynamic(Direction.kForward));
      driver.back().and(driver.b()).whileTrue(elevator.sysIdDynamic(Direction.kReverse));
      driver.start().and(driver.a()).whileTrue(elevator.sysIdQuasistatic(Direction.kForward));
      driver.start().and(driver.b()).whileTrue(elevator.sysIdQuasistatic(Direction.kReverse));

      driver.povRight().whileTrue(drivetrain.wheelRadiusCharacterization(1));
      driver.povLeft().whileTrue(drivetrain.wheelRadiusCharacterization(-1));

      driver.leftBumper().whileTrue(drivetrain.applyRequest(() ->
          point.withModuleDirection(new Rotation2d(0, 0))
      ));

    // driver.start().whileTrue(drivetrain.wheelRadiusCharacterization(1));
  }

  private void configureBindings() {
      // Note that X is defined as forward according to WPILib convention,
      // and Y is defined as to the left according to WPILib convention.
      drivetrain.setDefaultCommand(
          // Drivetrain will execute this command periodically
          drivetrain.applyRequest(() ->
              drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with Y (forward)
                  .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with X (left)
                  .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
          )
      );

      trg_driverDanger.and(driver.leftBumper()).whileTrue(
        Commands.parallel(
          drivetrain.applyRequest(() -> straightWheelsReq),
          Commands.runOnce(() ->  drivetrain.setNeutralMode(NeutralModeValue.Coast)
        ).finallyDo(() -> drivetrain.setNeutralMode(NeutralModeValue.Brake)))
      );

      // driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
      // driver.y().whileTrue(drivetrain.applyRequest(() ->
      //     point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
      // ));
      driver.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())); // reset the field-centric heading

      driver.rightBumper().onTrue(
        Commands.parallel(
          algae.toIdleCmd(),
          superstructure.forceIdle()
        )
      );

      Supplier<Command> leftTeleopAutoAlignCmdSupp = () -> {
        return drivetrain.moveToPose(
          Vision.getMostRealisticScorePose(drivetrain.getState().Pose, false),
          visionSim);
      };
      Supplier<Command> rightTeleopAutoAlignCmdSupp = () -> {
        return drivetrain.moveToPose(
          Vision.getMostRealisticScorePose(drivetrain.getState().Pose, true),
          visionSim);
      };

      trg_leftTeleopAutoAlign.whileTrue(
        Commands.repeatingSequence(
          new DeferredCommand(leftTeleopAutoAlignCmdSupp, Set.of(drivetrain))
        )
      );
      trg_rightTeleopAutoAlign.whileTrue(
        Commands.repeatingSequence(
          new DeferredCommand(rightTeleopAutoAlignCmdSupp, Set.of(drivetrain))
        )
      );
      trg_driverDanger.and(driver.rightTrigger()).onTrue(superstructure.forceShoot());
     
      trg_manipDanger.and(trg_intakeReq).onTrue(superstructure.forceStateToIntake());
      trg_manipDanger.and(trg_toL1).onTrue(superstructure.forceL1());
      trg_manipDanger.and(trg_toL2).onTrue(superstructure.forceL2());
      trg_manipDanger.and(trg_toL3).onTrue(superstructure.forceL3());
      trg_manipDanger.and(trg_toL4).onTrue(superstructure.forceL4());

      trg_manipDanger.and(manipulator.back()).debounce(1).onTrue(
        Commands.parallel(
          elevator.currentSenseHoming(),
          finger.currentSenseHoming(),
          algae.currentSenseHoming()
        ).andThen(superstructure.forceIdle())
      );

      manipulator.leftBumper().onTrue(superstructure.forceIdle());

      trg_deAlgae.and(trg_toL2).and(trg_manipDanger).onTrue(
        Commands.parallel(
          elevator.toHeightAlgae(() -> AlgaeHeight.L2),
          superstructure.baseAlgaeRemoval()
        )
      );
      trg_deAlgae.and(trg_toL3).and(trg_manipDanger).onTrue(
        Commands.parallel(
          elevator.toHeightAlgae(() -> AlgaeHeight.L3),
          superstructure.baseAlgaeRemoval()
        )
      );

    manipulator.y().and(manipulator.povUp())
    .onTrue(Commands.parallel(
      elevator.toHeight(EleHeight.CLIMB_UP.rotations),
      finger.fingerClimbDownCmd()
    ));

    manipulator.y().and(manipulator.povDown())
      .onTrue(elevator.toHeight(EleHeight.CLIMB_DOWN.rotations));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  /* WaltAutonBuilder thingies */
  private void configWaltAutonBuilder() {
    WaltAutonBuilder.cyclesChooser.onChange(cyclesConsumer);
    WaltAutonBuilder.startingPositionChooser.onChange(startingPositionConsumer);
    WaltAutonBuilder.startingHeightChooser.onChange(startingHeightConsumer);
    WaltAutonBuilder.firstScoringChooser.onChange(initialScoringPositionConsumer);
    WaltAutonBuilder.firstToHPStationChooser.onChange(initialHPStationConsumer);
  }

  private void driverRumble(double intensity) {
		if (!DriverStation.isAutonomous()) {
			driver.getHID().setRumble(RumbleType.kBothRumble, intensity);
		}
	}

  private void manipRumble(double intensity) {
		if (!DriverStation.isAutonomous()) {
			manipulator.getHID().setRumble(RumbleType.kBothRumble, intensity);
		} 
	}

  @Override
  public void robotInit(){
    WaltAutonBuilder.configureFirstCycle();
    configWaltAutonBuilder();
    
    addPeriodic(() -> superstructure.periodic(), 0.01);
    robotField.getObject("reefARobotLocation").setPose(FieldK.Reef.reefLocationToIdealRobotPoseMap.get(ReefLocs.REEF_A));
    robotField.getObject("reefBRobotLocation").setPose(FieldK.Reef.reefLocationToIdealRobotPoseMap.get(ReefLocs.REEF_B));
    robotField.getObject("reefCRobotLocation").setPose(FieldK.Reef.reefLocationToIdealRobotPoseMap.get(ReefLocs.REEF_C));
    robotField.getObject("reefDRobotLocation").setPose(FieldK.Reef.reefLocationToIdealRobotPoseMap.get(ReefLocs.REEF_D));
    robotField.getObject("reefERobotLocation").setPose(FieldK.Reef.reefLocationToIdealRobotPoseMap.get(ReefLocs.REEF_E));
    robotField.getObject("reefFRobotLocation").setPose(FieldK.Reef.reefLocationToIdealRobotPoseMap.get(ReefLocs.REEF_F));
    robotField.getObject("reefGRobotLocation").setPose(FieldK.Reef.reefLocationToIdealRobotPoseMap.get(ReefLocs.REEF_G));
    robotField.getObject("reefHRobotLocation").setPose(FieldK.Reef.reefLocationToIdealRobotPoseMap.get(ReefLocs.REEF_H));
    robotField.getObject("reefIRobotLocation").setPose(FieldK.Reef.reefLocationToIdealRobotPoseMap.get(ReefLocs.REEF_I));
    robotField.getObject("reefJRobotLocation").setPose(FieldK.Reef.reefLocationToIdealRobotPoseMap.get(ReefLocs.REEF_J));
    robotField.getObject("reefKRobotLocation").setPose(FieldK.Reef.reefLocationToIdealRobotPoseMap.get(ReefLocs.REEF_K));
    robotField.getObject("reefLRobotLocation").setPose(FieldK.Reef.reefLocationToIdealRobotPoseMap.get(ReefLocs.REEF_L));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    if (autonNotMade) {
      readyToMakeAuton = WaltAutonBuilder.nte_autonEntry.getBoolean(false);
    }

    if (readyToMakeAuton && autonNotMade) {
      waltAutonFactory = new WaltAutonFactory(
        elevator,
        autoFactory, 
        superstructure, 
        drivetrain,
        WaltAutonBuilder.startingPosition, 
        WaltAutonBuilder.getCycleScoringLocs(), 
        WaltAutonBuilder.getCycleEleHeights(), 
        WaltAutonBuilder.getCycleHPStations(),
        false
      );

      // dummy one
      // waltAutonFactory = new WaltAutonFactory(
      //   autoFactory, 
      //   superstructure, 
      //   StartingLocs.MID, 
      //   reefLocs, 
      //   heights, 
      //   hpStations
      // );

      AutonChooser.addPathsAndCmds(waltAutonFactory);
      autonNotMade = false;
    }

    if (beforeAuton) {
      if (numCycleChange) {
        WaltAutonBuilder.updateNumCycles();
        WaltAutonBuilder.configureCycles(); // dont need to call configureFirstCycle since the num of cycles chosen doesn't affect the preload cycle
        numCycleChange = false;
      }
      if (startingPositionChange) {
        WaltAutonBuilder.updateStartingPosition();
        WaltAutonBuilder.configureFirstCycle(); // changing the initial position affects the options given for scoring locs
        startingPositionChange = false;
      }
      if (initialHPStationChange) {
        WaltAutonBuilder.updateInitalHPStation();
        initialHPStationChange = false;
      }
      if (firstScoringPositionChange) {
        WaltAutonBuilder.updateInitialScoringPosition();
        firstScoringPositionChange = false;
      }
      if (startingHeightChange) {
        WaltAutonBuilder.updateStartingHeight();
        startingHeightChange = false;
      }
    }
    
    // loops through each camera and adds its pose estimation to the drivetrain pose estimator if required
    for (Vision camera : cameras) {
      Optional<EstimatedRobotPose> estimatedPoseOptional = camera.getEstimatedGlobalPose();
      if (estimatedPoseOptional.isPresent()) {
        EstimatedRobotPose estimatedRobotPose = estimatedPoseOptional.get();
        Pose2d estimatedRobotPose2d = estimatedRobotPose.estimatedPose.toPose2d();
        var ctreTime = Utils.fpgaToCurrentTime(estimatedRobotPose.timestampSeconds);
        drivetrain.addVisionMeasurement(estimatedRobotPose2d, ctreTime, camera.getEstimationStdDevs());
      }
    }

    robotField.getRobotObject().setPose(drivetrain.getStateCopy().Pose);
    log_robotPose.accept(drivetrain.getState().Pose);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  private Command autonCmdBuilder(Command chooserCommand) {
    return Commands.parallel(
          Commands.print("running autonCmdBuilder"),
          superstructure.autonPreloadReq(),
          algae.currentSenseHoming(),
          chooserCommand
      );
  }

  @Override
  public void autonomousInit() {
    Command chosen = AutonChooser.autoChooser.selectedCommandScheduler();
    m_autonomousCommand = autonCmdBuilder(chosen);

    if(m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    Commands.runOnce(() -> waltAutonFactory.autonTimer.stop());
    superstructure.forceIdle().schedule();
    algae.toIdleCmd().schedule();
    finger.fingerInCmd().schedule();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // if(!elevator.getIsHomed()) {
    //   elevator.currentSenseHoming().schedule();
    // }

    // if(!algae.getIsHomed()) {
    //   algae.currentSenseHoming().schedule();
    // }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {
    SwerveDriveState robotState = drivetrain.getState();
    Pose2d robotPose = robotState.Pose;
    visionSim.simulationPeriodic(robotPose);
    drivetrain.simulationPeriodic();

    // below is debug for swerve simulation. the farthest down one displays the module poses, but it's definitely bugged
    // Field2d debugField = visionSim.getSimDebugField();
    // simDebugField.getObject("EstimatedRobot").setPose(robotPose);
    // simDebugField.getObject("EstimatedRobotModules").setPoses(drivetrain.extractModulePoses(robotState));
  }
}