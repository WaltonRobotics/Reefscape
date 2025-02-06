// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
  private final Vision vision = new Vision();

  public final Superstructure superstructure =  new Superstructure(algae, coral, elevator, (intensity) -> driverRumble(intensity), driver.rightTrigger());

  private final AutoFactory autoFactory = drivetrain.createAutoFactory();
  private final WaltAutonFactory waltAutonFactory = new WaltAutonFactory(autoFactory, drivetrain, elevator);
  private final AutoChooser autoChooser = new AutoChooser();

  public Robot() {
    /* autossss */
    autoChooser.addRoutine("auton", () -> waltAutonFactory.getAuton());

    SmartDashboard.putData("Auto Chooser", autoChooser);

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
        // wrist position controls
        manipulator.y().onTrue(algae.toAngle(Algae.WristPosition.HOME));
        manipulator.x().onTrue(algae.toAngle(Algae.WristPosition.INTAKE));
        manipulator.b().onTrue(algae.toAngle(Algae.WristPosition.PROCESSOR_SHOOT));

        // intake controls
        // manipulator.rightTrigger()
        //     .whileTrue(algae.setWheelAction(Algae.IntakeSpeed.INTAKE));
        // manipulator.leftTrigger()
        //     .whileTrue(algae.setWheelAction(Algae.IntakeSpeed.PROCESSOR_SHOOT));
        


        driver.povDown().onTrue(elevator.toHome());
        driver.povLeft().onTrue(elevator.setPosition(Elevator.EleHeights.L1));
        driver.povRight().onTrue(elevator.setPosition(Elevator.EleHeights.L2));
        driver.povUp().onTrue(elevator.setPosition(Elevator.EleHeights.L3));
        driver.x().onTrue(elevator.setPosition(Elevator.EleHeights.L4));
        driver.y().onTrue(elevator.toCS());

        driver.a().onTrue(elevator.setPosition(Elevator.EleHeights.CLIMB_UP));
        driver.b().onTrue(elevator.toHome());

        drivetrain.registerTelemetry(logger::telemeterize);
  }

  private void driverRumble(double intensity) {
		if (!DriverStation.isAutonomous()) {
			driver.getHID().setRumble(RumbleType.kBothRumble, intensity);
		}
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
    superstructure.autonPreload();
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
    superstructure.resetAfterAuton();
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
    drivetrain.simulationPeriodic();
    Pose2d robotPose = drivetrain.getState().Pose;
    vision.simulationPeriodic(robotPose);

    Field2d debugField = vision.getSimDebugField();
    debugField.getObject("EstimatedRobot").setPose(robotPose);
    debugField.getObject("EstimatedRobotModules").setPoses(drivetrain.getModulePoses());
  }
}