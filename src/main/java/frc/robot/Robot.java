// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.Coralk.kCoralSpeed;

import java.util.function.Consumer;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autons.AutonChooser;
// import frc.robot.autons.AutonChooser;
import frc.robot.autons.SimpleAutons;
// import frc.robot.autons.AutonChooser.NumCycles;
import static frc.robot.autons.TrajsAndLocs.*;
import frc.robot.autons.WaltAutonFactory;
// import frc.robot.autons.WaltAutonFactory;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Algae.WristPos;
import frc.robot.subsystems.Elevator.EleHeight;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Superstructure;

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
  private final Elevator elevator = new Elevator();
  private final Algae algae;
  private final Superstructure superstructure;

  private Command m_autonomousCommand;
  private final AutoFactory autoFactory = drivetrain.createAutoFactory();
  private final SimpleAutons simpleAutons;
  private final WaltAutonFactory waltAutonFactory;

  private final Trigger trg_intakeReq = manipulator.rightBumper();
  private final Trigger trg_processorReq = manipulator.y();
  private final Trigger trg_shootReq = manipulator.rightTrigger();
  
  private final Trigger trg_toL1 = manipulator.povDown();
  private final Trigger trg_toL2 = manipulator.povRight();
  private final Trigger trg_toL3 = manipulator.povLeft();
  private final Trigger trg_toL4 = manipulator.povUp();

  private final Trigger trg_algaeIntake = manipulator.a();

  private boolean numCycleChange = false;
  private boolean startingPositionChange = false;
  private boolean firstScoringPositionChange = false;
  private boolean startingHeightChange = false;
  private boolean initialHPStationChange = false;
 
  
  // override button
  private final Trigger trg_manipDanger = manipulator.b();
  private final Trigger trg_forceIdleState = manipulator.leftBumper();

  private final Trigger trg_teleopScoreReq = driver.rightTrigger(); // IMPORTANT: CHANGE BACK TO DRIVER BUTTON

  public Robot() {
    DriverStation.silenceJoystickConnectionWarning(true);
    superstructure = new Superstructure(
      coral, 
      elevator, 
      trg_intakeReq,
      trg_toL1,
      trg_toL2,
      trg_toL3,
      trg_toL4,
      trg_teleopScoreReq,
      trg_forceIdleState,
      this::driverRumble);

    // algae = new Algae(
    //   manipulator.a(), 
    //   manipulator.leftTrigger(), 
    //   manipulator.y(), 
    //   manipulator.rightTrigger(), 
    //   manipulator.back(), 
    //   (intensity) -> manipRumble(intensity), 
    //   () -> manipulator.getRightY());

    algae = new Algae(
      trg_algaeIntake, 
      trg_processorReq,
      trg_shootReq,
      this::manipRumble);

    simpleAutons = new SimpleAutons(autoFactory, superstructure);
    waltAutonFactory = new WaltAutonFactory(
      autoFactory, 
      superstructure,
      StartingLocs.MID,
      ReefLocs.REEF_H,
      HPStation.HP_RIGHT);

    AutonChooser.addPathsAndCmds(waltAutonFactory);

    configureBindings();
    configureTestBindings();
  }

  private void configureTestBindings() {
    // driver.a().onTrue(
    //   Commands.sequence(
    //     algae.toAngle(WristPos.GROUND),
    //     algae.intake()
    //   )
    // ).onFalse(algae.toAngle(WristPos.HOME));
  
    manipulator.y().whileTrue(elevator.testVoltageControl(() -> manipulator.getLeftY()));
    // driver.x().whileTrue(algae.testVoltageControl(() -> driver.getLeftY()));

    // driver.x().onTrue(elevator.toHeight(Feet.of(1).in(Meters)));
    // driver.y().onTrue(elevator.toHeight(Inches.of(1).in(Meters)));

  }

  private void configureBindings() {
      // Note that X is defined as forward according to WPILib convention,
      // and Y is defined as to the left according to WPILib convention.
      drivetrain.setDefaultCommand(
          // Drivetrain will execute this command periodically
          drivetrain.applyRequest(() ->
              drive.withVelocityX(driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                  .withVelocityY(driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                  .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
          )
      );

      driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
      // driver.y().whileTrue(drivetrain.applyRequest(() ->
      //     point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
      // ));
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

      drivetrain.registerTelemetry(logger::telemeterize);

  }

  // private final Consumer<NumCycles> cyclesConsumer = numCycles -> {
  //   AutonChooser.m_cycles = numCycles;
  //   numCycleChange = true;
  // };
  // private final Consumer<StartingLocs> startingPositionConsumer = startingPosition -> {
  //   AutonChooser.startingPosition = startingPosition;
  //   startingPositionChange = true;
  // }  ;
  // private final Consumer<EleHeight> startingHeightConsumer = startingHeight -> {
  //   AutonChooser.startingHeight = startingHeight;
  //   numCycleChange = true;
  // }  ;
  // private final Consumer<ReefLocs> initialScoringPositionConsumer = scoringPosition -> {
  //   AutonChooser.scoringPosition = scoringPosition;
  //   firstScoringPositionChange = true;
  // }  ;
  // private final Consumer<HPStation> initalHPStationConsumer = hpStation -> {
  //   AutonChooser.hpStation = hpStation;
  //   initialHPStationChange = true;
  // };
  
  // private void mapAutonCommands() {
  //   AutonChooser.configureFirstCycle();
  // }

  // /* needed to continue choosing schtuffs */
  // private void configAutonChooser() {
  //   AutonChooser.cyclesChooser.onChange(cyclesConsumer);
  //   AutonChooser.startingPositionChooser.onChange(startingPositionConsumer);
  //   AutonChooser.startingHeightChooser.onChange(startingHeightConsumer);
  //   AutonChooser.firstScoringChooser.onChange(initialScoringPositionConsumer);
  //   AutonChooser.firstToHPStationChooser.onChange(initalHPStationConsumer);
  // }

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
    addPeriodic(() -> superstructure.periodic(), 0.01);
    // mapAutonCommands();
    // configAutonChooser();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = AutonChooser.autoChooser.selectedCommand();

    if(m_autonomousCommand != null) {
      Commands.parallel(
          superstructure.autonPreloadReq().alongWith(Commands.print("at preloadReq calling")),
          algae.currentSenseHoming(),
          Commands.sequence(
            Commands.run(()->{}).until(() -> elevator.getIsHomed()),
            m_autonomousCommand
            )
      ).schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    superstructure.stateToIdle();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // if(!elevator.getIsHomed()) {
      // elevator.currentSenseHoming().schedule();
    // }

    // if(!algae.getIsHomed()) {
      // algae.currentSenseHoming().schedule();
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
  public void simulationPeriodic() {}
}