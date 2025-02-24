// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.Coralk.kCoralSpeed;

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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autons.AutonChooser;
import frc.robot.autons.TrajsAndLocs;
import frc.robot.autons.TrajsAndLocs.ReefLocs;
import frc.robot.autons.TrajsAndLocs.StartingLocs;
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
  private final Algae algae;
  private final Superstructure superstructure;

  private final AutoFactory autoFactory = drivetrain.createAutoFactory();
  // private final WaltAutonFactory waltAutonFactory = new WaltAutonFactory(autoFactory);

  private final Trigger trg_intakeReq = manipulator.rightBumper();
  
  private final Trigger trg_eleToScoreReq = manipulator.y().and(manipulator.b());

  private final Trigger trg_toL1 = manipulator.povDown();
  private final Trigger trg_toL2 = manipulator.povRight();
  private final Trigger trg_toL3 = manipulator.povLeft();
  private final Trigger trg_toL4 = manipulator.povUp();
  
  // override button
  private final Trigger trg_manipDanger = manipulator.b();
  private final Trigger trg_driverDanger = driver.b();
  private final Trigger trg_forceIntakeState = trg_manipDanger.and(manipulator.rightBumper());
  private final Trigger trg_forceScoreState = trg_manipDanger.and(manipulator.leftTrigger());
  private final Trigger trg_forceIdleState = trg_manipDanger.and(manipulator.leftBumper());

  private final Trigger trg_climbUp = manipulator.a().and(manipulator.povUp());
  private final Trigger trg_climbDown = manipulator.a().and(manipulator.povDown());

  private final Trigger trg_teleopScoreReq = driver.rightTrigger();

  public Robot() {
    superstructure = new Superstructure(
      coral, 
      elevator, 
      trg_intakeReq,
      trg_toL4,
      trg_teleopScoreReq,
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
      new Trigger(() -> false), 
      new Trigger(() -> false), 
      new Trigger(() -> false), 
      new Trigger(() -> false), 
      null, 
      () -> 0);

    configureBindings();
    // configureTestBindings();
  }

  private void configureTestBindings() {
    // driver.a().onTrue(
    //   Commands.sequence(
    //     algae.toAngle(WristPos.GROUND),
    //     algae.intake()
    //   )
    // ).onFalse(algae.toAngle(WristPos.HOME));
  
    // driver.y().whileTrue(elevator.testVoltageControl(() -> driver.getLeftY()));
    // driver.x().whileTrue(algae.testVoltageControl(() -> driver.getLeftY()));

    // driver.x().onTrue(elevator.toHeight(Feet.of(1).in(Meters)));
    // driver.y().onTrue(elevator.toHeight(Inches.of(1).in(Meters)));

    driver.leftBumper().whileTrue(coral.automaticCoralIntake());
    driver.leftTrigger().whileTrue(coral.score());
    driver.rightTrigger().whileTrue(coral.runWheelsAlgaeRemovalCmd());

    driver.b().whileTrue(coral.fastIntake());
    driver.x().onTrue(coral.fingerOutCmd());
    driver.y().onTrue(coral.fingerInCmd());
    driver.a().whileTrue(coral.algaeIntake());

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
      driver.y().whileTrue(drivetrain.applyRequest(() ->
          point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
      ));
      driver.x().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())); // reset the field-centric heading

      trg_toL1.onTrue(Commands.runOnce(() -> superstructure.requestEleHeight(() -> EleHeight.L1)));
      trg_toL2.onTrue(Commands.runOnce(() -> superstructure.requestEleHeight(() -> EleHeight.L2)));
      trg_toL3.onTrue(Commands.runOnce(() -> superstructure.requestEleHeight(() -> EleHeight.L3)));
      trg_toL4.onTrue(Commands.runOnce(() -> superstructure.requestEleHeight(() -> EleHeight.L4)));

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
    m_autonomousCommand = null; // TODO: fill out

    if (m_autonomousCommand != null) {
      Commands.parallel(
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