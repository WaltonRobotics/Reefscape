// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.lang.StackWalker.Option;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.datalog.BooleanLogEntry;
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
import frc.robot.autoalign.AutoAlignUtils;
import frc.robot.autoalign.MovingAutoAlign;
import frc.robot.autons.AutonChooser;
import frc.robot.autons.WaltAutonBuilder;
import frc.robot.autons.TrajsAndLocs.HPStation;
import frc.robot.autons.TrajsAndLocs.ReefLocs;
import frc.robot.autons.TrajsAndLocs.StartingLocs;
import static frc.robot.autons.TrajsAndLocs.ReefLocs.*;

import frc.robot.autons.WaltAutonFactory;
import frc.robot.generated.TunerConstants;
import frc.util.Elastic;
import frc.util.WaltLogger;
import frc.util.Elastic.NotificationLevel;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;
import frc.robot.subsystems.Elevator.AlgaeHeight;
import frc.robot.subsystems.Elevator.EleHeight;
import frc.robot.vision.Vision;
import frc.robot.vision.VisionSim;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Algae.State;

public class Robot extends TimedRobot {

  private final double kMaxTranslationSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private final double kMaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  private final double kMaxHighAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond);
  private final double kSlowSpeed = kMaxTranslationSpeed * .1;
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
  private final Coral coral = new Coral();
  private final Finger finger = new Finger();
  private final Elevator elevator = new Elevator();
  private final Funnel funnel = new Funnel();
  private final Algae algae;
  private final Superstructure superstructure;

  private Command m_autonomousCommand;
  // VisionSim could probably be static or a singleton instead of this reference mess but that's extra work to potentially break something
  private final VisionSim visionSim = new VisionSim();
  private final Vision eleForwardsCam = new Vision(VisionK.kElevatorForwardsCamName, VisionK.kElevatorForwardsCamSimVisualName,
    VisionK.kElevatorForwardsCamRoboToCam, visionSim, VisionK.kEleForwardCamSimProps);
  private final Vision lowerRightCam = new Vision(VisionK.kLowerRightCamName, VisionK.kLowerRightCamSimVisualName,
    VisionK.kLowerRightCamRoboToCam, visionSim, VisionK.kLowerRightCamSimProps);

  // this should be updated with all of our cameras
  private final Vision[] cameras = {eleForwardsCam, lowerRightCam};  // lower right cam removed readded and ready to rumble

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

  private final Trigger trg_toHPReq = manipulator.rightBumper();
  private final Trigger trg_intakeReq = manipulator.rightTrigger();
  
  private final Trigger trg_toL1 = manipulator.povDown();
  private final Trigger trg_toL2 = manipulator.povRight();
  private final Trigger trg_toL3 = manipulator.povLeft();
  private final Trigger trg_toL4 = manipulator.povUp();

  private final Trigger trg_teleopScoreReq = driver.rightTrigger(); 

  private final Trigger trg_algaeIntake = manipulator.a();
  private final Trigger trg_processorReq = manipulator.y();
  private final Trigger trg_shootReq = manipulator.rightTrigger();
  private final Trigger trg_deAlgae = manipulator.leftTrigger();

  private final Trigger trg_climbPrep = manipulator.y().and(manipulator.povUp());
  private final Trigger trg_climbBump = manipulator.start();
  private final Trigger trg_climbLockingIn = manipulator.y().and(manipulator.povDown());

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
  private Optional<WaltAutonFactory> waltAutonFactory;
  private boolean autonMade = false;

  // autons
  public Optional<WaltAutonFactory> auton_right;

  public Optional<WaltAutonFactory> auton_left;

  public boolean auton_midOne;

  public void updateStaticField() {
    Robot.robotField = visionSim.getSimDebugField();
  }

  public Robot() {
    SignalLogger.start();
    DriverStation.silenceJoystickConnectionWarning(true);
    if (Robot.isReal()) {
      lastGotTagMsmtTimer.start();
      superstructure = new Superstructure(
      coral,
      finger,
      elevator,
      Optional.of(eleForwardsCam),
      funnel,
      trg_toHPReq.and(trg_manipDanger.negate()),
      trg_intakeReq,
      trg_toL1,
      trg_toL2,
      trg_toL3,
      trg_toL4,
      trg_teleopScoreReq,
      trg_deAlgae.and(trg_toL2),
      trg_deAlgae.and(trg_toL3),
      trg_climbPrep,
      manipulator.start(),
      trg_climbLockingIn,
      trg_inOverride,
      new Trigger(() -> false),
      new Trigger(() -> false),
      this::driverRumble);
    } else {
      superstructure = new Superstructure(
      coral,
      finger,
      elevator,
      Optional.empty(),
      funnel,
      trg_toHPReq.and(trg_manipDanger.negate()),
      trg_intakeReq,
      trg_toL1,
      trg_toL2,
      trg_toL3,
      trg_toL4,
      trg_teleopScoreReq,
      trg_deAlgae.and(trg_toL2),
      trg_deAlgae.and(trg_toL3),
      trg_climbPrep,
      trg_climbBump,
      trg_climbLockingIn,
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

    drivetrain.registerTelemetry(logger::telemeterize);


    updateStaticField();
    configureBindings();
    // configureTestBindings();

    // --- AUTONS
    auton_midOne = false;

    auton_left = Optional.of(autonFactoryFactory(
      StartingLocs.LEFT, 
      List.of(REEF_J, REEF_K, REEF_L, REEF_A),
      List.of(EleHeight.L4, EleHeight.L4, EleHeight.L4, EleHeight.L4),
      List.of(HPStation.HP_LEFT, HPStation.HP_LEFT, HPStation.HP_LEFT, HPStation.HP_LEFT)
      )
    );

    auton_right  = Optional.of(autonFactoryFactory(
        StartingLocs.RIGHT,
        List.of(REEF_E, REEF_D, REEF_C, REEF_B),
        List.of(EleHeight.L4, EleHeight.L4, EleHeight.L4, EleHeight.L4),
        List.of(HPStation.HP_RIGHT, HPStation.HP_RIGHT, HPStation.HP_RIGHT, HPStation.HP_RIGHT)
      )
    );
  }

  private final Runnable cameraSnapshotFunc = () -> {
    for (Vision camera : cameras) {
      camera.takeBothSnapshots();
    }
  };

  public WaltAutonFactory autonFactoryFactory(
    StartingLocs startLoc, List<ReefLocs> scoreLocs,
    List<EleHeight> heights, List<HPStation> hpStations) {
      return new WaltAutonFactory(
        elevator, drivetrain.autoFactory, superstructure, drivetrain, funnel, //cameraSnapshotFunc,
        startLoc, new ArrayList<>(scoreLocs), new ArrayList<>(heights), new ArrayList<>(hpStations)
      );
  }

  Command autoAlignCmd(boolean rightReef) {
    return MovingAutoAlign.autoAlignWithIntermediateTransformUntilInTolerances(
      drivetrain, 
      () -> AutoAlignUtils.getMostLikelyScorePose(drivetrain.getState(), rightReef), 
      () -> SharedAutoAlignK.kIntermediatePoseTransform
    ).alongWith(Commands.runOnce(cameraSnapshotFunc));
  }

  // checks for finger in unsafe place
  // no it doesnt lol. it prolly should tho.
  private Command resetEverythingCheck() {
    return Commands.parallel(
        algae.toIdleCmd(),
        superstructure.forceIdle()
      );
  }

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

      driver.back().and(driver.a()).whileTrue(elevator.sysIdDynamic(Direction.kForward));
      driver.back().and(driver.b()).whileTrue(elevator.sysIdDynamic(Direction.kReverse));
      driver.start().and(driver.a()).whileTrue(elevator.sysIdQuasistatic(Direction.kForward));
      driver.start().and(driver.b()).whileTrue(elevator.sysIdQuasistatic(Direction.kReverse));

      driver.povRight().whileTrue(drivetrain.wheelRadiusCharacterization(1));
      driver.povLeft().whileTrue(drivetrain.wheelRadiusCharacterization(-1));

      driver.leftBumper().whileTrue(drivetrain.applyRequest(() ->
          point.withModuleDirection(new Rotation2d(0, 0))
      ));

      driver.y().whileTrue(
        drivetrain.swervePIDTuningSeq(
          FieldConstants.kReefRobotLocationPoseMap.get(ReefLocs.REEF_E),
          robotField
          )
        );

      trg_leftTeleopAutoAlign.whileTrue(
        autoAlignCmd(false)
      );
      trg_rightTeleopAutoAlign.whileTrue(
        autoAlignCmd(true)
      );

    // driver.start().whileTrue(drivetrain.wheelRadiusCharacterization(1));
  }

  private Command driveCommand() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    // Drivetrain will execute this command periodically

    //define slewrate limiter here
    return drivetrain.applyRequest(() -> {
      var angularRate = driver.leftTrigger().getAsBoolean() ? 
        kMaxHighAngularRate : kMaxAngularRate;
    
      boolean slow = elevator.getPulleyRotations() >= (8.451660 + (0.169 / 2));
      double actualMaxTrSpeed = slow ? kSlowSpeed : kMaxTranslationSpeed;
      //reset slewratelimiter here
      var driverXVelo = -driver.getLeftY() * actualMaxTrSpeed;
      var driverYVelo = -driver.getLeftX() * actualMaxTrSpeed;
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

    driver.rightBumper().onTrue(
      Commands.parallel(
        resetEverythingCheck()
      )
    );

    trg_leftTeleopAutoAlign.whileTrue(
      autoAlignCmd(false)
    );
    trg_rightTeleopAutoAlign.whileTrue(
      autoAlignCmd(true)
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
        superstructure.algaeRemoval()
      )
    );
    trg_deAlgae.and(trg_toL3).and(trg_manipDanger).onTrue(
      Commands.parallel(
        elevator.toHeightAlgae(() -> AlgaeHeight.L3),
        superstructure.algaeRemoval()
      )
    );    

    manipulator.y()
      .onTrue(algae.changeStateCmd(State.HOME));
    
    trg_manipDanger.and(trg_toHPReq)
      .whileTrue(funnel.ejectFlap());

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

    superstructure.periodic();
    
    // loops through each camera and adds its pose estimation to the drivetrain pose estimator if required
    for (Vision camera : cameras) {
      Optional<EstimatedRobotPose> estimatedPoseOptional = camera.getEstimatedGlobalPose();
      if (estimatedPoseOptional.isPresent()) {
        EstimatedRobotPose estimatedRobotPose = estimatedPoseOptional.get();
        Pose2d estimatedRobotPose2d = estimatedRobotPose.estimatedPose.toPose2d();
        var ctreTime = Utils.fpgaToCurrentTime(estimatedRobotPose.timestampSeconds);
        drivetrain.addVisionMeasurement(estimatedRobotPose2d, ctreTime, camera.getEstimationStdDevs());
        lastGotTagMsmtTimer.restart();
      }
    }

    boolean visionSeenPastSec = !lastGotTagMsmtTimer.hasElapsed(1);
    log_visionSeenPastSecond.accept(visionSeenPastSec);
    // double rio6VCurrent = RobotController.getCurrent6V();
    // log_rio6VRailCurrent.accept(rio6VCurrent);
  }

  @Override
  public void disabledInit() {
    // set the auton to rightside by default if nothing is picked and auton j starts
    waltAutonFactory = auton_right;
    m_autonomousCommand = autonCmdBuilder(waltAutonFactory.get().generateAuton().cmd());

    WaltAutonBuilder.initialize();
  }

  @Override
  public void disabledPeriodic() {
    if (Superstructure.nte_autonOnCart.get(false)) {
      Constants.kTestingAutonOnCart = true;
    } else {
      Constants.kTestingAutonOnCart = false;
    }
    
    if (!autonMade) {
      // ALL OF THE COMMENTED OUT ITEMS ARE FOR DEBUGGING THE PUBLISHER BOOLEAN VALUES

      if (WaltAutonBuilder.sub_right.getAsBoolean()) {
        waltAutonFactory = auton_right;
        Elastic.sendNotification(new Elastic.Notification(NotificationLevel.INFO, "Auton Path DEFINED", "Right Auton Defined"));
        WaltAutonBuilder.pub_right.set(false);
        WaltAutonBuilder.pub_autonName.set("Right_4_Piece (E-L4, D-L4, C-L4, B-L4)");
        // Elastic.sendNotification(new Elastic.Notification(NotificationLevel.INFO, "right" + WaltAutonBuilder.sub_right.getAsBoolean(), ""));
      }
      if (WaltAutonBuilder.sub_left.getAsBoolean()) {
        waltAutonFactory = auton_left;
        Elastic.sendNotification(new Elastic.Notification(NotificationLevel.INFO, "Auton Path DEFINED", "Left Auton Defined"));
        WaltAutonBuilder.pub_left.set(false);
        WaltAutonBuilder.pub_autonName.set("Left_4_Piece (J-L4, K-L4, L-L4, A-L4)");
        // Elastic.sendNotification(new Elastic.Notification(NotificationLevel.INFO, "left " + WaltAutonBuilder.sub_left.getAsBoolean(), ""));
      }
      if (WaltAutonBuilder.sub_midOne.getAsBoolean()) {
        auton_midOne = true;
        Elastic.sendNotification(new Elastic.Notification(NotificationLevel.INFO, "Auton Path DEFINED", "MidOne Auton Defined"));
        WaltAutonBuilder.pub_midOne.set(false);
        WaltAutonBuilder.pub_autonName.set("Mid_G_Piece (REEF_G L4)");
        // Elastic.sendNotification(new Elastic.Notification(NotificationLevel.INFO, "midOne " + WaltAutonBuilder.sub_midOne.getAsBoolean(), ""));
      }

      if (WaltAutonBuilder.sub_makeAuton.getAsBoolean() && waltAutonFactory.isPresent()) {
        if (auton_midOne) {
          m_autonomousCommand = autonCmdBuilder(waltAutonFactory.get().midAuton().cmd());
          Elastic.sendNotification(new Elastic.Notification(NotificationLevel.INFO, "Auton MADE", "MidOne Auton MADE"));
        } else {
          m_autonomousCommand = autonCmdBuilder(waltAutonFactory.get().generateAuton().cmd());
          Elastic.sendNotification(new Elastic.Notification(NotificationLevel.INFO, "Auton MADE", "SIDE Auton MADE"));
        }

        WaltAutonBuilder.pub_makeAuton.set(false);
        // System.out.println("make auton " + WaltAutonBuilder.sub_makeAuton);
        // Elastic.sendNotification(new Elastic.Notification(NotificationLevel.INFO, "make auton " + WaltAutonBuilder.sub_makeAuton.getAsBoolean(), ""));
        autonMade = true;
        WaltAutonBuilder.pub_autonMade.set(true);
      }
    }

    if (WaltAutonBuilder.sub_clearAll.getAsBoolean() && waltAutonFactory.isPresent()) {
      waltAutonFactory = Optional.empty();
      Elastic.sendNotification(new Elastic.Notification(NotificationLevel.INFO, "AUTON CLEARED", "PLEASE MAKE AN AUTON"));
      m_autonomousCommand = Commands.print("========== No Auton Command Selected ==========");

      WaltAutonBuilder.pub_clearAll.set(false);
      WaltAutonBuilder.pub_makeAuton.set(false);
      WaltAutonBuilder.pub_midOne.set(false);
      WaltAutonBuilder.pub_left.set(false);
      WaltAutonBuilder.pub_right.set(false);
      WaltAutonBuilder.pub_autonName.set("No Auton Made");
      // System.out.println("clear all sub" + WaltAutonBuilder.sub_clearAll.getAsBoolean());
      // Elastic.sendNotification(new Elastic.Notification(NotificationLevel.INFO, "clear all " + WaltAutonBuilder.sub_clearAll.getAsBoolean(), ""));
      // System.out.println("make auto sub" + WaltAutonBuilder.sub_makeAuton.getAsBoolean());
      // Elastic.sendNotification(new Elastic.Notification(NotificationLevel.INFO, "make auto " + WaltAutonBuilder.sub_makeAuton.getAsBoolean(), ""));
      // System.out.println("midOne sub" + WaltAutonBuilder.sub_midOne.getAsBoolean());
      // Elastic.sendNotification(new Elastic.Notification(NotificationLevel.INFO, "mid one" + WaltAutonBuilder.sub_midOne.getAsBoolean(), ""));
      // System.out.println("left sub" + WaltAutonBuilder.sub_left.getAsBoolean());
      // Elastic.sendNotification(new Elastic.Notification(NotificationLevel.INFO, "left " + WaltAutonBuilder.sub_left.getAsBoolean(), ""));
      // System.out.println("right sub" + WaltAutonBuilder.sub_right.getAsBoolean());
      // Elastic.sendNotification(new Elastic.Notification(NotificationLevel.INFO, "right" + WaltAutonBuilder.sub_right.getAsBoolean(), ""));

      autonMade = false;
      WaltAutonBuilder.pub_autonMade.set(false);
    }
  }

  @Override
  public void disabledExit() {
    if (waltAutonFactory.isEmpty()) {
      waltAutonFactory = auton_right;
      m_autonomousCommand = autonCmdBuilder(waltAutonFactory.get().generateAuton().cmd());
    }
  }

  private Command autonCmdBuilder(Command chooserCommand) {
    return Commands.parallel(
          Commands.print("running autonCmdBuilder"),
          // funnel.ejectFlap(),
          superstructure.autonPreloadReq(),
          algae.currentSenseHoming(),
          superstructure.simHasCoralToggle(),
          chooserCommand          
      );
  }

  @Override
  public void autonomousInit() {
    if (waltAutonFactory.isPresent()) {
      waltAutonFactory.get().startAutonTimer();
    }

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
    if (waltAutonFactory.isPresent()) {
      Commands.runOnce(() -> waltAutonFactory.get().autonTimer.stop());
    }
    superstructure.forceToIntake().schedule();
    algae.toIdleCmd().schedule();
    finger.inCmd().schedule();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {

  }

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