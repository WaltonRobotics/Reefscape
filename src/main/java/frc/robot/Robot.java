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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autons.AutonChooser;
import frc.robot.autons.TrajsAndLocs;
import frc.robot.autons.TrajsAndLocs.HPStation;
import frc.robot.autons.TrajsAndLocs.ReefLocs;
import frc.robot.autons.TrajsAndLocs.StartingLocs;
import frc.robot.autons.WaltAutonFactory;
import frc.robot.autons.AutonChooser.NumCycles;
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
  private final Algae algae = new Algae();
  private final Coral coral = new Coral();
  private final Elevator elevator = new Elevator();
  private final Superstructure superstructure;

  // public final Superstructure superstructure =  new Superstructure(algae, coral, elevator, (intensity) -> driverRumble(intensity), driver.rightTrigger());

  private final AutoFactory autoFactory = drivetrain.createAutoFactory();
  //private final WaltAutonFactory waltAutonFactory = new WaltAutonFactory(autoFactory, drivetrain, elevator);
  private final AutoChooser autoChooser = new AutoChooser();

  private void mapAutonCommands(){

    AutonChooser.setDefaultAuton(TrajsAndLocs.StartingLocs.MID);
    AutonChooser.setDefaultHPStation(TrajsAndLocs.HPStation.HP_LEFT);

    AutonChooser.assignNumCycles(NumCycles.CYCLE_1, "Cycle 1");
    AutonChooser.assignNumCycles(NumCycles.CYCLE_2, "Cycle 2");
    AutonChooser.assignNumCycles(NumCycles.CYCLE_3, "Cycle 3");
    AutonChooser.assignNumCycles(NumCycles.CYCLE_4, "Cycle 4");

    AutonChooser.assignStartingPosition(TrajsAndLocs.StartingLocs.RIGHT, "right");
    AutonChooser.assignStartingPosition(TrajsAndLocs.StartingLocs.MID, "mid");
    AutonChooser.assignStartingPosition(TrajsAndLocs.StartingLocs.LEFT, "left");
    AutonChooser.assignHPStation(TrajsAndLocs.HPStation.HP_LEFT, "human player left");
    AutonChooser.assignHPStation(TrajsAndLocs.HPStation.HP_RIGHT, "human player right");

   AutonChooser.chooseEleHeight("starting height chooser");
  }

  public Robot() {
    /* autossss */
    // autoChooser.addRoutine("auton", () -> waltAutonFactory.getAuton());

    // all score request buttons must be OR'd here
    // TODO: make named triggers for each for reuse
    Trigger eleHeightReq = 
      manipulator.povDown()
      .or(manipulator.povUp())
      .or(manipulator.povLeft())
      .or(manipulator.povRight());

    // TODO: review intake/score reqs with drivers
    superstructure = new Superstructure(
      coral, elevator, 
      manipulator.x(), 
      eleHeightReq, driver.leftTrigger(), 
      this::driverRumble, this::manipRumble
    );

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
        driver.rightTrigger().whileTrue(coral.setCoralMotorAction(kCoralSpeed));
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
          .whileTrue(coral.setCoralMotorAction(kCoralSpeed));
        
        // elevator controls
        // todo: leftBumper, home request
        manipulator.leftBumper().onTrue(elevator.toPosition(EleHeight.HOME));
        
        // manipulator.povDown().onTrue(superstructure.requestToScore(EleHeight.L1));
        // manipulator.povRight().onTrue(superstructure.requestToScore(EleHeight.L2));
        // manipulator.povLeft().onTrue(superstructure.requestToScore(EleHeight.L3));
        // manipulator.povUp().onTrue(superstructure.requestToScore(EleHeight.L4));
        
        // climber controls
        // TODO: ask superstructure if in idle first
        manipulator.a().and(manipulator.povUp()).onTrue(elevator.toPosition(EleHeight.CLIMB_UP));
        manipulator.a().and(manipulator.povDown()).onTrue(elevator.toPosition(EleHeight.CLIMB_DOWN));
        
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

  private void manipRumble(double intensity) {
		if (!DriverStation.isAutonomous()) {
			manipulator.getHID().setRumble(RumbleType.kBothRumble, intensity);
		}
	}

  @Override
  public void robotInit(){
    mapAutonCommands();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if(AutonChooser.firstScoringChosen.get() == null){
      AutonChooser.chooseFirstScoring();
    }
    if(AutonChooser.hpToReefChosen.get() == null || AutonChooser.reefToHPChosen.get() == null){
      AutonChooser.cycleIterations();
    }
    // if(AutonChooser.hpToReefChosen.get() != null && AutonChooser.reefToHPChosen.get() != null){
    //   AutonChooser.getSelectedValues();
    // }
  
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
