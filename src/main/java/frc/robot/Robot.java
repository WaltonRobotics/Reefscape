// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autons.AutonChooser;
import frc.robot.autons.TrajsAndLocs;
import frc.robot.autons.TrajsAndLocs.HPStation;
import frc.robot.autons.TrajsAndLocs.ReefLocs;
import frc.robot.autons.TrajsAndLocs.StartingLocs;
import frc.robot.autons.WaltAutonFactory;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;
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
  private final Algae algae = new Algae();
  private final Coral coral = new Coral();
  private final Elevator elevator = new Elevator();

  // public final Superstructure superstructure =  new Superstructure(algae, coral, elevator, (intensity) -> driverRumble(intensity), driver.rightTrigger());

  private final AutoFactory autoFactory = drivetrain.createAutoFactory();
  private final WaltAutonFactory waltAutonFactory = new WaltAutonFactory(autoFactory, drivetrain, elevator);
  private final AutoChooser autoChooser = new AutoChooser();

  private void mapAutonCommands(){
    AutonChooser.setDefaultAuton(TrajsAndLocs.StartingLocs.MID);
    AutonChooser.assignPosition(TrajsAndLocs.StartingLocs.RIGHT, "right");
    AutonChooser.assignPosition(TrajsAndLocs.StartingLocs.MID, "mid");
    AutonChooser.assignPosition(TrajsAndLocs.StartingLocs.LEFT, "left");
    AutonChooser.chooseFirstScoring();
    AutonChooser.chooseHPStation(TrajsAndLocs.HPStation.HP_LEFT, "human player left");
    AutonChooser.chooseHPStation(TrajsAndLocs.HPStation.HP_RIGHT, "human player right");
  }

  public Robot() {
    /* autossss */
    // autoChooser.addRoutine("auton", () -> waltAutonFactory.getAuton());

    configureBindings();
  }

  private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        //joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //joystick.rightBumper().whileTrue(drivetrain.wheelRadiusCharacterization(1));
        //joystick.rightTrigger().whileTrue(drivetrain.wheelRadiusCharacterization(-1));

        /*TODO: test to see if this actually works */
        // driver controls (not sure abt alignment inputs)
        driver.rightTrigger().whileTrue(coral.score());
        driver.rightTrigger().whileTrue(algae.setWheelAction(Algae.IntakeSpeed.PROCESSOR_SHOOT));
        
        // wrist position controls
        manipulator.a().onTrue(algae.toAngle(Algae.WristPosition.HOME));
        manipulator.x().onTrue(algae.toAngle(Algae.WristPosition.INTAKE));
        manipulator.y().onTrue(algae.toAngle(Algae.WristPosition.PROCESSOR_SHOOT));

        // intake controls
        manipulator.leftTrigger()
          .whileTrue(algae.setWheelAction(Algae.IntakeSpeed.INTAKE));
        manipulator.rightTrigger()
          .whileTrue(algae.setWheelAction(Algae.IntakeSpeed.PROCESSOR_SHOOT));
        manipulator.leftTrigger().and(manipulator.b())
          .whileTrue(coral.intake());
        
        // elevator controls
        manipulator.leftBumper().onTrue(elevator.toHome());
        manipulator.povDown().onTrue(elevator.setPosition(Elevator.EleHeights.L1));
        manipulator.povRight().onTrue(elevator.setPosition(Elevator.EleHeights.L2));
        manipulator.povLeft().onTrue(elevator.setPosition(Elevator.EleHeights.L3));
        manipulator.povUp().onTrue(elevator.setPosition(Elevator.EleHeights.L4));
        manipulator.rightStick().whileTrue(elevator.setPosition(manipulator.getRightX())); // might need lambda?

        // climber controls
        manipulator.a().and(manipulator.povUp()).onTrue(elevator.setPosition(Elevator.EleHeights.CLIMB_UP));
        manipulator.a().and(manipulator.povDown()).onTrue(elevator.setPosition(Elevator.EleHeights.CLIMB_DOWN));
        
        //testing buttons
        // driver.rightBumper().whileTrue(drivetrain.wheelRadiusCharacterization(1));
        // driver.rightTrigger().whileTrue(drivetrain.wheelRadiusCharacterization(-1));

        drivetrain.registerTelemetry(logger::telemeterize);
  }

  private void driverRumble(double intensity) {
		if (!DriverStation.isAutonomous()) {
			driver.getHID().setRumble(RumbleType.kBothRumble, intensity);
		}
	}

  @Override
  public void robotInit(){
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    mapAutonCommands();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // superstructure.autonPreload();
    m_autonomousCommand = autoChooser.selectedCommand();

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
    // superstructure.resetAfterAuton();
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
