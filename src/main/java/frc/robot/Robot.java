// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.RomiDrivetrain;
import frc.robot.subsystems.SubsystemLidar;
import frc.robot.subsystems.SubsystemPathExec;
import frc.robot.subsystems.map.GoogleCartographer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private final XboxController m_armController = new XboxController(1);

  private Command m_autonomousCommand;

  private final RomiDrivetrain m_romiDrivetrain = new RomiDrivetrain();
  private SubsystemLidar lidar;
  private SubsystemPathExec pathExec;
  private GoogleCartographer cartographer;

  private RobotContainer m_robotContainer;
  Thread m_visionThread;
  private CvSource cvSource;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    /*
     * new Thread(() -> {
     * Mat frame = Imgcodecs.imread("TESTIMG.png");
     * 
     * while (true) {
     * MatOfByte matOfByte = new MatOfByte();
     * Imgproc.resize(frame, frame, new Size(640, 480));
     * 
     * Imgcodecs.imencode(".jpg", frame, matOfByte);
     * cvSource.putFrame(frame);
     * try {
     * Thread.sleep(500);
     * } catch (InterruptedException e) {
     * e.printStackTrace();
     * }
     * }
     * })
     * .start();
     */

    cvSource = CameraServer.putVideo("MJPEG Server", 200, 200);

    ShuffleboardTab tab = Shuffleboard.getTab("SLAM");
    tab
        .add("SLAM map", cvSource)
        .withWidget(BuiltInWidgets.kCameraStream)
        .withSize(3, 2)
        .withPosition(0, 0);

    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    if (lidar != null) {
      lidar.setScanning(false);
      lidar.closeLidar();
    }
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    cartographer = new GoogleCartographer();
    lidar = new SubsystemLidar(cartographer.getCallback(), true, "/dev/ttyUSB0");
    lidar.setScanning(true);

    pathExec = new SubsystemPathExec(true, m_romiDrivetrain);
    cartographer.initiate("src/main/java/frc/robot/configuration/cartographer_config",
        "cartographer_config_main.lua", false, false, 10);
    pathExec.setDefaultCommand(
        new RunCommand(() -> pathExec.tick(cartographer.getCartographerMapData(), cvSource), pathExec));

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    m_romiDrivetrain.setDefaultCommand(
        new RunCommand(
            () -> m_romiDrivetrain.arcadeDrive(0, m_armController.getRawAxis(0)),
            m_romiDrivetrain));

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
