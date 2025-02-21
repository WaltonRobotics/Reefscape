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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  private final Telemetry logger = new Telemetry(MaxSpeed);

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController manipulator = new CommandXboxController(1);

  public final Swerve drivetrain = TunerConstants.createDrivetrain();
  private final Coral coral = new Coral();
  private final Elevator elevator = new Elevator();

  private final AutoFactory autoFactory = drivetrain.createAutoFactory();
  private final WaltAutonFactory waltAutonFactory = new WaltAutonFactory(autoFactory);
  private boolean cycleChange = false;
  /* AutonChooser trigs */
  private final Consumer<StartingLocs> startLocConsumer = startLoc -> {
    AutonChooser.startLocChosen = startLoc;
    AutonChooser.chooseFirstScoring();
  };
  private final Consumer<HPStation> hpStationConsumer = hpStation -> {
    AutonChooser.hpStationChosen = hpStation;
    cycleChange = true;
  };
  private final Consumer<ReefLocs> hpToReefConsumer = hpToReef -> {
    AutonChooser.hpToReefChosen = hpToReef;
    cycleChange = true;
  };
  private final Consumer<HPStation> reefToHPConsumer = reefToHP -> {
    AutonChooser.reefToHPChosen = reefToHP;
    cycleChange = true;
  };
  private final Consumer<NumCycles> cyclesConsumer = numCycles -> {
    AutonChooser.cyclesChosen = numCycles;
    cycleChange = true;
  };

  private final Trigger trg_teleopEleHeightReq = 
    manipulator.povDown() //L1
    .or(manipulator.povRight()) // L2
    .or(manipulator.povLeft()) // L3
    .or(manipulator.povUp()); // L4
  // sameer wanted b to be his ele override button also, so i created a trigger to check that he didnt mean to press any other override when using b
  private final Trigger trg_eleOverride =
    manipulator.rightBumper().negate()
    .and(manipulator.leftTrigger().negate())
    .and(trg_teleopEleHeightReq.negate());
  // override button
  private final Trigger trg_manipDanger = manipulator.b();
  private final Trigger trg_driverDanger = driver.b();

  private final Superstructure superstructure =  new Superstructure(
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

  public static final Field2d field2d = new Field2d();

  private Command m_autonomousCmd;

  public Robot() {
    DriverStation.silenceJoystickConnectionWarning(true);
    if(Robot.isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
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

    drivetrain.registerTelemetry(logger::telemeterize);

  }

  private void mapAutonCommands(){
    AutonChooser.setDefaultAuton(TrajsAndLocs.StartingLocs.MID);
    AutonChooser.setDefaultHPStation(TrajsAndLocs.HPStation.HP_LEFT);

    AutonChooser.assignNumCycles(NumCycles.CYCLE_1, "Cycle 1");
    AutonChooser.assignNumCycles(NumCycles.CYCLE_2, "Cycle 2");
    AutonChooser.assignNumCycles(NumCycles.CYCLE_3, "Cycle 3");
    AutonChooser.assignNumCycles(NumCycles.CYCLE_4, "Cycle 4");

    AutonChooser.assignStartingPosition(TrajsAndLocs.StartingLocs.RIGHT, "right");
    AutonChooser.assignStartingPosition(TrajsAndLocs.StartingLocs.LEFT, "left");

    AutonChooser.assignHPStation(TrajsAndLocs.HPStation.HP_RIGHT, "human player right");

    AutonChooser.assignStartingHeight(Elevator.EleHeight.L1, "L1");
    AutonChooser.assignStartingHeight(Elevator.EleHeight.L2, "L2");
    AutonChooser.assignStartingHeight(Elevator.EleHeight.L3, "L3");
    AutonChooser.assignStartingHeight(Elevator.EleHeight.L4, "L4");
  }

  /* needed to continue choosing schtuffs */
  private void configAutonChooser() {
    AutonChooser.startingPositionChooser.onChange(startLocConsumer);
    AutonChooser.hpStationChooser.onChange(hpStationConsumer);
    AutonChooser.hpToReefChooser.onChange(hpToReefConsumer);
    for(int i = 0; i < AutonChooser.reefToHPChoosers.size(); i++){
      AutonChooser.reefToHPChoosers.get(i).onChange(reefToHPConsumer);
    }
    AutonChooser.cyclesChooser.onChange(cyclesConsumer);
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
    configAutonChooser();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if(cycleChange){
      AutonChooser.cycleIterations();
      cycleChange = false;
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
    m_autonomousCmd = waltAutonFactory.generateAuton(
      drivetrain, 
      superstructure, 
      AutonChooser.getChosenStart(), 
      AutonChooser.getChosenFirstReef(), 
      AutonChooser.getStartingHeight(), 
      AutonChooser.getChosenFirstHP(), 
      AutonChooser.getCycles());

    if(m_autonomousCmd != null) {
      m_autonomousCmd.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
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