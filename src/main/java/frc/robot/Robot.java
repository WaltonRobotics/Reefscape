// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.lang.StackWalker.Option;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.util.datalog.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.MovingAutoAlignK;
import frc.robot.Constants.SharedAutoAlignK;
import frc.robot.Constants.VisionK;

import frc.robot.generated.TunerConstants;
import frc.util.Elastic;
import frc.util.WaltLogger;
import frc.util.Elastic.NotificationLevel;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;
// import frc.robot.vision.Vision;
// import frc.robot.vision.VisionSim;
import frc.robot.subsystems.*;
// import frc.robot.subsystems.Algae.State;

public class Robot extends TimedRobot {

  private final double kMaxTranslationSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private final double kMaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  private final double kMaxHighAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond);
  private final Telemetry logger = new Telemetry(kMaxTranslationSpeed);

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(kMaxTranslationSpeed * 0.1).withRotationalDeadband(kMaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController manipulator = new CommandXboxController(1);

  public final Swerve drivetrain = TunerConstants.createDrivetrain();
  // private final Algae algae;

  private Command m_autonomousCommand;
  // // VisionSim could probably be static or a singleton instead of this reference mess but that's extra work to potentially break something
  // private final VisionSim visionSim = new VisionSim();
  // private final Vision eleForwardsCam = new Vision(VisionK.kElevatorForwardsCamName, VisionK.kElevatorForwardsCamSimVisualName,
  //   VisionK.kElevatorForwardsCamRoboToCam, visionSim, VisionK.kEleForwardCamSimProps);
  // private final Vision lowerRightCam = new Vision(VisionK.kLowerRightCamName, VisionK.kLowerRightCamSimVisualName,
  //   VisionK.kLowerRightCamRoboToCam, visionSim, VisionK.kLowerRightCamSimProps);

  // // this should be updated with all of our cameras
  // private final Vision[] cameras = {eleForwardsCam, lowerRightCam};  // lower right cam removed readded and ready to rumble

  private final DoubleLogger log_stickDesiredFieldX = WaltLogger.logDouble("Swerve", "stick desired teleop x");
  private final DoubleLogger log_stickDesiredFieldY = WaltLogger.logDouble("Swerve", "stick desired teleop y");
  private final DoubleLogger log_stickDesiredFieldZRot = WaltLogger.logDouble("Swerve", "stick desired teleop z rot");
  private final DoubleLogger log_fieldX = WaltLogger.logDouble("Swerve", "calculated field x speed");
  private final DoubleLogger log_fieldY = WaltLogger.logDouble("Swerve", "calculate field y speed");

  private final BooleanLogger log_povDown = WaltLogger.logBoolean("DPad", "povDown");
  private final BooleanLogger log_povRight = WaltLogger.logBoolean("DPad", "povRight");
  private final BooleanLogger log_povLeft = WaltLogger.logBoolean("DPad", "povLeft");
  private final BooleanLogger log_povUp = WaltLogger.logBoolean("DPad", "povUP");
  
  private final Trigger trg_leftTeleopAutoAlign = driver.x();
  private final Trigger trg_rightTeleopAutoAlign = driver.a();

  // private final Trigger trg_teleopEleHeightReq;
  // sameer wanted b to be his ele override button also, so i created a trigger to check that he didnt mean to press any other override when using b
  // private final Trigger trg_eleOverride;


  private final Trigger trg_teleopScoreReq = driver.rightTrigger(); 

  private final Trigger trg_algaeIntake = manipulator.a();
  private final Trigger trg_processorReq = manipulator.y();
  private final Trigger trg_shootReq = manipulator.rightTrigger();

  // simulation
  private final Trigger trg_simBotBeamBreak = manipulator.leftStick();
  private final Trigger trg_simTopBeamBreak = manipulator.rightStick();

  // override button
  private final Trigger trg_driverDanger = driver.b();
  private final Trigger trg_manipDanger = manipulator.b();
  private final Trigger trg_inOverride = trg_manipDanger.or(trg_driverDanger);

  private final SwerveRequest straightWheelsReq = new SwerveRequest.PointWheelsAt().withModuleDirection(new Rotation2d());

  private final DoubleLogger log_rio6VRailCurrent = WaltLogger.logDouble("Rio", "6VRailCurrent");

  // istg if you touch this outside of updateStaticField
  public static Field2d robotField = null;
  private final Timer lastGotTagMsmtTimer = new Timer();
  private final BooleanLogger log_visionSeenPastSecond = new BooleanLogger("Robot", "VisionSeenLastSec");

  // --- AUTONCHOOSER
  private boolean autonMade = false;

  // autons
  public boolean auton_midOne;

  // public void updateStaticField() {
  //   Robot.robotField = visionSim.getSimDebugField();
  // }

  public Robot() {
    SignalLogger.start();
    DriverStation.silenceJoystickConnectionWarning(true);

    drivetrain.registerTelemetry(logger::telemeterize);

    // algae = new Algae(
    //   trg_algaeIntake, 
    //   new Trigger(() -> false), 
    //   trg_shootReq, 
    //   this::manipRumble
    // );

    // updateStaticField();
    configureBindings();
    // configureTestBindings();
  }

  // private final Runnable cameraSnapshotFunc = () -> {
  //   for (Vision camera : cameras) {
  //     camera.takeBothSnapshots();
  //   }
  // };


  // Command autoAlignCmd(boolean rightReef) {
  //   return MovingAutoAlign.autoAlignWithIntermediateTransformUntilInTolerances(
  //     drivetrain, 
  //     () -> AutoAlignUtils.getMostLikelyScorePose(drivetrain.getState(), rightReef), 
  //     () -> SharedAutoAlignK.kIntermediatePoseTransform
  //   ).alongWith(Commands.runOnce(cameraSnapshotFunc));
  // }

  // checks for finger in unsafe place
  // no it doesnt lol. it prolly should tho.
  // private Command resetAlgaeCheck() {
  //   return Commands.parallel(
  //       algae.toIdleCmd()
  //     );
  // }

  private void configureTestBindings() {
    drivetrain.setDefaultCommand(
          // Drivetrain will execute this command periodically
          drivetrain.applyRequest(() ->
              drive.withVelocityX(-driver.getLeftY() * kMaxTranslationSpeed) // Drive forward with Y (forward)
                  .withVelocityY(-driver.getLeftX() * kMaxTranslationSpeed) // Drive left with X (left)
                  .withRotationalRate(-driver.getRightX() * kMaxAngularRate) // Drive counterclockwise with negative X (left)
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

      driver.povRight().whileTrue(drivetrain.wheelRadiusCharacterization(1));
      driver.povLeft().whileTrue(drivetrain.wheelRadiusCharacterization(-1));

      driver.leftBumper().whileTrue(drivetrain.applyRequest(() ->
          point.withModuleDirection(new Rotation2d(0, 0))
      ));

      // driver.y().whileTrue(
      //   drivetrain.swervePIDTuningSeq(
      //     FieldConstants.kReefRobotLocationPoseMap.get(ReefLocs.REEF_E),
      //     robotField
      //     )
      //   );

      // trg_leftTeleopAutoAlign.whileTrue(
      //   autoAlignCmd(false)
      // );
      // trg_rightTeleopAutoAlign.whileTrue(
      //   autoAlignCmd(true)
      // );

    // driver.start().whileTrue(drivetrain.wheelRadiusCharacterization(1));
  }

  private Command driveCommand() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    // Drivetrain will execute this command periodically
    return drivetrain.applyRequest(() -> {
      var angularRate = driver.leftTrigger().getAsBoolean() ? 
        kMaxHighAngularRate : kMaxAngularRate;
    
      var driverXVelo = -driver.getLeftY() * kMaxTranslationSpeed;
      var driverYVelo = -driver.getLeftX() * kMaxTranslationSpeed;
      var driverYawRate = -driver.getRightX() * angularRate;

      log_stickDesiredFieldX.accept(driverXVelo);
      log_stickDesiredFieldY.accept(driverYVelo);
      log_stickDesiredFieldZRot.accept(driverYawRate);
        
      return drive
        .withVelocityX(driverXVelo) // Drive forward with Y (forward)
        .withVelocityY(driverYVelo) // Drive left with X (left)
        .withRotationalRate(driverYawRate); // Drive counterclockwise with negative X (left)
    });
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(driveCommand());

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

    // driver.rightBumper().onTrue(
    //   Commands.parallel(
    //     resetAlgaeCheck()
    //   )
    // );

    // trg_leftTeleopAutoAlign.whileTrue(
    //   autoAlignCmd(false)
    // );
    // trg_rightTeleopAutoAlign.whileTrue(
    //   autoAlignCmd(true)
    // );

    // trg_manipDanger.and(manipulator.back()).debounce(1).onTrue(
    //   Commands.parallel(
    //     algae.currentSenseHoming()
    //   )
    // );

    // manipulator.y()
    //   .onTrue(algae.changeStateCmd(State.HOME));
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
    // addPeriodic(() -> superstructure.periodic(), 0.01);

    SmartDashboard.putData("ReefPoses", FieldConstants.kReefPosesField2d);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    
    // loops through each camera and adds its pose estimation to the drivetrain pose estimator if required
    // for (Vision camera : cameras) {
    //   Optional<EstimatedRobotPose> estimatedPoseOptional = camera.getEstimatedGlobalPose();
    //   if (estimatedPoseOptional.isPresent()) {
    //     EstimatedRobotPose estimatedRobotPose = estimatedPoseOptional.get();
    //     Pose2d estimatedRobotPose2d = estimatedRobotPose.estimatedPose.toPose2d();
    //     var ctreTime = Utils.fpgaToCurrentTime(estimatedRobotPose.timestampSeconds);
    //     drivetrain.addVisionMeasurement(estimatedRobotPose2d, ctreTime, camera.getEstimationStdDevs());
    //     lastGotTagMsmtTimer.restart();
    //   }
    // }

    // boolean visionSeenPastSec = !lastGotTagMsmtTimer.hasElapsed(1);
    // log_visionSeenPastSecond.accept(visionSeenPastSec);
    // double rio6VCurrent = RobotController.getCurrent6V();
    // log_rio6VRailCurrent.accept(rio6VCurrent);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    // CommandScheduler.getInstance().schedule(algae.toIdleCmd());
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
    // visionSim.simulationPeriodic(robotPose);
    drivetrain.simulationPeriodic();

    // below is debug for swerve simulation. the farthest down one displays the module poses, but it's definitely bugged
    // Field2d debugField = visionSim.getSimDebugField();
    // simDebugField.getObject("EstimatedRobot").setPose(robotPose);
    // simDebugField.getObject("EstimatedRobotModules").setPoses(drivetrain.extractModulePoses(robotState));
  }
}