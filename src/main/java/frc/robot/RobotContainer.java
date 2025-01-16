// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve"));

  private SendableChooser<Command> autoChooser;

  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;
    // Create an LED pattern that will display a rainbow across
  // all hues at maximum saturation and half brightness
  private final LEDPattern rainbow = LEDPattern.rainbow(255, 128);

  // Our LED strip has a density of 120 LEDs per meter
  private static final Distance kLedSpacing = Meters.of(1 / 120.0);

  // Create a new pattern that scrolls the rainbow pattern across the LED strip, moving at a speed
  // of 1 meter per second.
  private final LEDPattern scrollingRainbow =
      rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);

  private final PowerDistribution pdp = new PowerDistribution();

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the rotational velocity 
  // buttons are quick rotation positions to different ways to face
  // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
  Command closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                  () -> Constants.SPEED_SCALING_3*MathUtil.applyDeadband(-driverXbox.getLeftY(),
                                                                OperatorConstants.DEADBAND),
                                  () -> Constants.SPEED_SCALING_3*MathUtil.applyDeadband(-driverXbox.getLeftX(),
                                                                OperatorConstants.DEADBAND),
                                  () -> Constants.SPEED_SCALING_3*MathUtil.applyDeadband(-driverXbox.getRightX(),
                                                                OperatorConstants.RIGHT_X_DEADBAND),
                                                          ()-> (driverXbox.getHID().getPOV() == 0),
                                                          ()-> (driverXbox.getHID().getPOV() == 180),
                                                          ()-> (driverXbox.getHID().getPOV() == 90),
                                                          ()-> (driverXbox.getHID().getPOV() == 270))
                                                          .withName("Absolute Advanced");

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative direct angle input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(() -> driverXbox.getRightX() * -1,
                                                                                             () -> driverXbox.getRightY() * -1)
                                                           .headingWhile(true);


  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  // This doesn't do what we want in 2025.1.1
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                                    .allianceRelativeControl(false);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle)
                                                    .withName("Direct Angle");


  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity)
                                                       .withName("Angular Velocity");


  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented)
                                                        .withName("Robot Oriented");


  // Drive field oriented angular velocity using PathPlanner set point generator
  Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                   () -> -driverXbox.getLeftY(),
                                                                   () -> -driverXbox.getLeftX())
                                                               .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
                                                               .deadband(OperatorConstants.DEADBAND)
                                                               .scaleTranslation(0.8)
                                                               .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                     .withControllerHeadingAxis(() -> Math.sin(
                                                                                                    driverXbox.getRawAxis(
                                                                                                        2) * Math.PI) * (Math.PI * 2),
                                                                                                () -> Math.cos(
                                                                                                    driverXbox.getRawAxis(
                                                                                                        2) * Math.PI) *
                                                                                                      (Math.PI * 2))
                                                                     .headingWhile(true);

  Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);

  Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);

  // Available drive modes to select from the chooser
  enum DriveMode {
    DIRECT_ANGLE,
    ABSOLUTE_ADVANCED,
    ANGULAR_VELOCITY,
    ROBOT_ORIENTED
  }

  // Drive mode chooser to allow changing mode each time TeleOp is enabled. Default is used if 
  // chooser is not opened. Pull up on dashboard or sim GUI (SmartDashborad/SendableChooser[0])
  SendableChooser<DriveMode> driveChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
        // Setup an addressable LED strip on a PWM channel
    setupLEDs();
    
    // Setup Auto commands and chooser
    NamedCommands.registerCommand(
        "SayHello",
        new InstantCommand(()->DataLogManager.log("Hello ...")));

    NamedCommands.registerCommand(
        "SayGoodbye",
        new InstantCommand(()->DataLogManager.log("... Goodbye")));

    NamedCommands.registerCommand(
        "Set_LED_Red",
        new InstantCommand(()->solidColor(Color.kRed)));

    NamedCommands.registerCommand(
        "Set_LED_Green",
        
        new InstantCommand(()->solidColor(Color.kGreen)));

    NamedCommands.registerCommand(
        "Set_LED_Blue",
        new InstantCommand(()->solidColor(Color.kBlue)));


    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(autoChooser);

    // Publish drive subsystem data for dashboard and logging
    SmartDashboard.putData(drivebase);
    
    // Publish PDP data for dashboard and logging
    SmartDashboard.putData(pdp);

    // Configure the trigger bindings
    configureBindings();

    // Setup chooser for selecting drive mode
    driveChooser.setDefaultOption("Drive Mode - AngularVelocity", DriveMode.ANGULAR_VELOCITY);
    driveChooser.addOption("Drive Mode - Direct Angle", DriveMode.DIRECT_ANGLE);
    driveChooser.addOption("Drive Mode - Advanced", DriveMode.ABSOLUTE_ADVANCED);
    driveChooser.addOption("Drive Mode - Robot Oriented", DriveMode.ROBOT_ORIENTED);
    SmartDashboard.putData(driveChooser);

    // Set default drive mode to one that isn't direct angle to avoid transient after auto
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {

    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity); // Overrides drive command above!

      driverXbox.b().whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else
    {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyroWithAlliance)));
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.b().whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              );
      driverXbox.y().whileTrue(Commands.none());
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().toggleOnTrue(driveRobotOrientedAngularVelocity);
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    return autoChooser.getSelected();
  }

  /**
   * Use this to set the teleop drive mode by setting the default drive command.
   *
   */
  public void setDriveMode()
  {
    switch (driveChooser.getSelected()) {
      case DIRECT_ANGLE:
        drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
        return;

      case ROBOT_ORIENTED:
        drivebase.setDefaultCommand(driveRobotOrientedAngularVelocity);
        return;

      case ABSOLUTE_ADVANCED:
        drivebase.setDefaultCommand(closedAbsoluteDriveAdv);
        return;

      case ANGULAR_VELOCITY:
      default:
        drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
        return;

    }
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  // Setup an interface for an addressable LED strip
  private void setupLEDs(){
    led = new AddressableLED(0);

    // Create the buffer. Start empty output
    // Length is expensive to set, so only set it once, then just update data
    ledBuffer = new AddressableLEDBuffer(30);
    led.setLength(ledBuffer.getLength());

    // Set the data
    led.setData(ledBuffer);
    led.start();
  }

  // Set all LEDs in the strip to a solid color
  private void solidRGB(int red, int green, int blue) {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      ledBuffer.setRGB(i, red, green, blue);
    }
    led.setData(ledBuffer);
  }

  // Set all LEDs in the strip to a solid color
  public void solidColor(Color color) {
    // Create an LED pattern that sets the entire strip to solid a color
    LEDPattern red = LEDPattern.solid(color);

    // Apply the LED pattern to the data buffer
    red.applyTo(ledBuffer);
    led.setData(ledBuffer);
  }

  public void rainbow() {
    // Update the buffer with the rainbow animation
    scrollingRainbow.applyTo(ledBuffer);
    // Set the LEDs
    led.setData(ledBuffer);
  }
}
