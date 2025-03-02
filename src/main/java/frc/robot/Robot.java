// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.Coralk.kCoralSpeed;

import java.util.Optional;
import java.util.Set;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.Choreo;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FieldK;
import frc.robot.Constants.VisionK;
import frc.robot.autons.AutonChooser;
import frc.robot.autons.TrajsAndLocs;
import frc.robot.autons.TrajsAndLocs.ReefLocs;
import frc.robot.autons.TrajsAndLocs.StartingLocs;
import frc.robot.autons.WaltAutonFactory;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Elevator.EleHeight;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Superstructure;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

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
  private final Elevator elevator = new Elevator();
  // VisionSim could probably be static or a singleton instead of this reference mess but that's extra work to potentially break something
  private final VisionSim visionSim = new VisionSim();
  private final Vision eleForwardsCam = new Vision(VisionK.kElevatorForwardsCamName, VisionK.kElevatorForwardsCamSimVisualName,
    VisionK.kElevatorForwardsCamRoboToCam, visionSim, VisionK.kEleForwardCamSimProps);
  private final Superstructure superstructure;

  // this should be updated with all of our cameras
  private final Vision[] cameras = {eleForwardsCam};

  private final AutoFactory autoFactory = drivetrain.createAutoFactory();
  private final WaltAutonFactory waltAutonFactory = new WaltAutonFactory(autoFactory);

  private final Trigger trg_teleopEleHeightReq;
  // sameer wanted b to be his ele override button also, so i created a trigger to check that he didnt mean to press any other override when using b
  private final Trigger trg_eleOverride;
  // override button
  private final Trigger trg_manipDanger;
  private final Trigger trg_driverDanger;

  public Robot() {
    // pov is the same thing as dpad right?
    trg_teleopEleHeightReq = manipulator.povDown() //L1
      .or(manipulator.povRight()) // L2
      .or(manipulator.povLeft()) // L3
      .or(manipulator.povUp()); // L4

    trg_eleOverride = 
      manipulator.rightBumper().negate()
      .and(manipulator.leftTrigger().negate())
      .and(trg_teleopEleHeightReq.negate());
    
    trg_manipDanger = manipulator.b();
    trg_driverDanger = driver.b();

    superstructure = new Superstructure(
      coral, 
      elevator, 
      manipulator.rightBumper(), 
      trg_teleopEleHeightReq,
      driver.rightTrigger(), 
      trg_manipDanger.and(manipulator.rightBumper()),
      trg_manipDanger.and(manipulator.leftTrigger()), 
      trg_manipDanger.and(trg_teleopEleHeightReq),
      trg_driverDanger.and(driver.rightTrigger()), 
      manipulator.leftBumper(),
      manipulator.a().and(manipulator.povUp()),
      manipulator.a().and(manipulator.povDown()),
      trg_manipDanger.and(trg_eleOverride),
      () -> manipulator.getLeftY(),
      (intensity) -> driverRumble(intensity), 
      (intensity) -> manipRumble(intensity));

    configureBindings();
  }

  private void configureBindings() {
      // Note that X is defined as forward according to WPILib convention,
      // and Y is defined as to the left according to WPILib convention.
      drivetrain.setDefaultCommand(
          // Drivetrain will execute this command periodically
          drivetrain.applyRequest(() ->
              drive.withVelocityX(driver.getLeftY() * MaxSpeed) // Drive forward with Y (forward)
                  .withVelocityY(driver.getLeftX() * MaxSpeed) // Drive left with X (left)
                  .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
          )
      );

      driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
      driver.y().whileTrue(drivetrain.applyRequest(() ->
          point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
      ));
      driver.x().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())); // reset the field-centric heading

      /* 
       * programmer buttons
       * make sure u comment out when not in use
       */
      // Run SysId routines when holding back/start and X/Y.
      // Note that each routine should be run exactly once in a single log.
      //driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
      //driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
      //driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
      //driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
      //driver.povRight().whileTrue(drivetrain.wheelRadiusCharacterization(1));
      //driver.povLeft().whileTrue(drivetrain.wheelRadiusCharacterization(-1));

    driver.leftBumper().onTrue(new DeferredCommand(() -> drivetrain.followTrajectory(
        drivetrain.generateTrajectory(eleForwardsCam.getReefScorePose(false))
    ), Set.of(drivetrain)));
    driver.rightBumper().onTrue(new DeferredCommand(() -> drivetrain.followTrajectory(
        drivetrain.generateTrajectory(eleForwardsCam.getReefScorePose(true))
    ), Set.of(drivetrain)));

      drivetrain.registerTelemetry(logger::telemeterize);
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
    Field2d simDebugField = visionSim.getSimDebugField();
    simDebugField.getObject("reefARobotLocation").setPose(FieldK.Reef.reefLocationToIdealRobotPoseMap.get(ReefLocs.REEF_A));
    simDebugField.getObject("reefBRobotLocation").setPose(FieldK.Reef.reefLocationToIdealRobotPoseMap.get(ReefLocs.REEF_B));
    simDebugField.getObject("reefCRobotLocation").setPose(FieldK.Reef.reefLocationToIdealRobotPoseMap.get(ReefLocs.REEF_C));
    simDebugField.getObject("reefDRobotLocation").setPose(FieldK.Reef.reefLocationToIdealRobotPoseMap.get(ReefLocs.REEF_D));
    simDebugField.getObject("reefERobotLocation").setPose(FieldK.Reef.reefLocationToIdealRobotPoseMap.get(ReefLocs.REEF_E));
    simDebugField.getObject("reefFRobotLocation").setPose(FieldK.Reef.reefLocationToIdealRobotPoseMap.get(ReefLocs.REEF_F));
    simDebugField.getObject("reefGRobotLocation").setPose(FieldK.Reef.reefLocationToIdealRobotPoseMap.get(ReefLocs.REEF_G));
    simDebugField.getObject("reefHRobotLocation").setPose(FieldK.Reef.reefLocationToIdealRobotPoseMap.get(ReefLocs.REEF_H));
    simDebugField.getObject("reefIRobotLocation").setPose(FieldK.Reef.reefLocationToIdealRobotPoseMap.get(ReefLocs.REEF_I));
    simDebugField.getObject("reefJRobotLocation").setPose(FieldK.Reef.reefLocationToIdealRobotPoseMap.get(ReefLocs.REEF_J));
    simDebugField.getObject("reefKRobotLocation").setPose(FieldK.Reef.reefLocationToIdealRobotPoseMap.get(ReefLocs.REEF_K));
    simDebugField.getObject("reefLRobotLocation").setPose(FieldK.Reef.reefLocationToIdealRobotPoseMap.get(ReefLocs.REEF_L));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    // loops through each camera and adds its pose estimation to the drivetrain pose estimator if required
    for (Vision camera : cameras) {
      Optional<EstimatedRobotPose> estimatedPoseOptional = camera.getEstimatedGlobalPose();
      if (estimatedPoseOptional.isPresent()) {
        EstimatedRobotPose estimatedRobotPose = estimatedPoseOptional.get();
        Pose2d estimatedRobotPose2d = estimatedRobotPose.estimatedPose.toPose2d();
        drivetrain.addVisionMeasurement(estimatedRobotPose2d, estimatedRobotPose.timestampSeconds);
      }
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = drivetrain.testAuton();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

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
    // debugField.getObject("EstimatedRobot").setPose(robotPose);
    // debugField.getObject("EstimatedRobotModules").setPoses(drivetrain.extractModulePoses(robotState));
  }
}