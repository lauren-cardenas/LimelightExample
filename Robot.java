/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.*;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final XboxController m_driverController = new XboxController(Map.DRIVER_CONTROLLER);
  private final XboxController m_operatorController = new XboxController(Map.OPERATOR_CONTROLLER);

  //Motors
  private final SpeedControllerGroup m_leftMotors =
    new SpeedControllerGroup(new WPI_VictorSPX(Map.LEFT_FRONT_MOTOR), new WPI_VictorSPX(Map.LEFT_REAR_MOTOR));
  private final SpeedControllerGroup m_rightMotors =
    new SpeedControllerGroup(new WPI_VictorSPX(Map.RIGHT_FRONT_MOTOR), new WPI_VictorSPX(Map.RIGHT_REAR_MOTOR));
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  //Encoders
  private final Encoder m_leftEncoder = new Encoder(Map.leftEnc1, Map.leftEnc2);
  private final Encoder m_rightEncoder = new Encoder(Map.rightEnc1, Map.rightEnc2);

  //Auto
  private NetworkTableEntry m_maxSpeed;

  //Limelight
  private boolean m_LimelightHasValidTarget = false;
  private double m_LimelightDriveCommand = 0.0;
  private double m_LimelightSteerCommand = 0.0;


  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    //Drivebase tab for Shuffleboard
    ShuffleboardTab driveBaseTab = Shuffleboard.getTab("Drivebase");
    driveBaseTab.add("GTA Drive", m_robotDrive);
    ShuffleboardLayout encoders = driveBaseTab.getLayout("List Layout", "Encoders").withPosition(0,0).withSize(2,2);

    //Creates a "Configuration" tab with a 'max speed' widget for autonomous
    //The widget will be placed in the second column and row and will be two columns wide
    m_maxSpeed = Shuffleboard.getTab("Configuration")
      .add("Max Speed", 1)
      .withWidget("Number Slider")
      .withPosition(1, 1)
      .withSize(2, 1)
      .getEntry();

    encoders.add("Left Encoder", m_leftEncoder);
    encoders.add("Right Encoder", m_rightEncoder);
    m_leftEncoder.setDistancePerPulse((Math.PI * Map.wheelDiameter) / Map.encoderCPR);
    m_rightEncoder.setDistancePerPulse((Math.PI * Map.wheelDiameter) / Map.encoderCPR);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    //Read the value of the 'max speed' widget from the dashboard
    m_robotDrive.setMaxOutput(m_maxSpeed.getDouble(1.0));
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    //driver controls
    /*
    double triggerVal = m_driverController.getTriggerAxis(Hand.kRight) - m_driverController.getTriggerAxis(Hand.kLeft);
    double stick = m_driverController.getX(Hand.kLeft) * Map.TURNING_RATE;
    
    m_robotDrive.tankDrive((triggerVal - stick) * Map.DRIVING_SPEED, (triggerVal + stick) * Map.DRIVING_SPEED);

    */

    //limelight tracking

    Update_Limelight_Tracking();

    double steer = m_driverController.getX(Hand.kRight);
    double drive = -m_driverController.getY(Hand.kLeft);
    boolean auto = m_driverController.getAButton();

    steer *= 0.70;
    drive *= 0.70;

    if (auto)
    {
      if (m_LimelightHasValidTarget)
      {
        m_robotDrive.arcadeDrive(m_LimelightDriveCommand, m_LimelightSteerCommand);
      }
      else
      {
        m_robotDrive.arcadeDrive(0.0,0.0);
      }
    }
    else
    {
      m_robotDrive.arcadeDrive(drive,steer);
    }

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  /**
   * This function implements a simple method of generating driving and steering commands
   * based on the tracking data from a limelight camera.
   */

  public void Update_Limelight_Tracking()
  {

    //These numbers must be tuned for your robot!  Be careful!
    final double STEER_K = 0.03;    // how hard to turn toward the target
    final double DRIVE_K = 0.26;    // how hard to drive fwd toward the target
    final double DESIRED_TARGET_AREA = 13.0;  //Area of the target when the robot reaches the wall
    final double MAX_DRIVE = 0.7;   // Simple speed limit so we don't drive too fast

    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

    if (tv < 1.0)
    {
      m_LimelightHasValidTarget = false;
      m_LimelightDriveCommand = 0.0;
      m_LimelightSteerCommand = 0.0;
      return;
    }

    m_LimelightHasValidTarget = true;

    //Start with proportional steering
    double steer_cmd = tx * STEER_K;
    m_LimelightSteerCommand = steer_cmd;

    //Try to drive forward until the target area reaches our desired area
    double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

    //Don't let the robot drive too fast into the goal
    if (drive_cmd > MAX_DRIVE)
    {
      drive_cmd = MAX_DRIVE;
    }
    m_LimelightDriveCommand = drive_cmd;

  }

}
