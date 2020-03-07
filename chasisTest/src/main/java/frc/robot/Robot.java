/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.EntryListenerFlags;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  DifferentialDrive m_robotDrive;

  Joystick m_stick;
  Timer m_timer;
  WPI_TalonSRX left1;
  WPI_TalonSRX left2;
  WPI_TalonSRX right1;
  WPI_TalonSRX right2;

  VictorSP motor;

  final int kTimeoutMs = 30;
  int rightPos = 0;
  int leftPos = 0;
  double errA, errC;

  double uly = 0;
  double ury = 0;
  double centerX = 159.5;
  double distance = 0;

  double x = 0;
  double y = 0;

  //motor = new VictorSP(1);
  


  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_stick = new Joystick(0);
    m_timer = new Timer();
    left1 = new WPI_TalonSRX(0);
    left2 = new WPI_TalonSRX(2);
    right1 = new WPI_TalonSRX(1);
    right2 = new WPI_TalonSRX(3);
    errA = 0;
    errC = 0;
    

    motor = new VictorSP(1);
    motor.set(1);
    
   // NetworkTableInstance inst = NetworkTableInstance.getDefault();

   // NetworkTable table = inst.getTable("datatable");


    right1.configFactoryDefault();
    left1.configFactoryDefault();
    
    right2.follow(right1);
    left2.follow(left1);
    right1.setInverted(false);
    right2.setInverted(false);
    left1.setInverted(false);
    left2.setInverted(false);
    right1.setSensorPhase(true);
    left1.setSensorPhase(false);





    m_robotDrive = new DifferentialDrive(left1, right1);

    // left1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,	0, kTimeoutMs);
    // right1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, kTimeoutMs);
    // right1.getSensorCollection().setQuadraturePosition(0, kTimeoutMs);
    // left1.getSensorCollection().setQuadraturePosition(0, kTimeoutMs);

  }

  @Override
  public void robotPeriodic() {
    // motor.set(1);
  }

  /**
   * This function is run once each time the robot enters autonomous mode.
   */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();


    
    
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
    // SmartDashboard.putNumber("uly", uly);
    // SmartDashboard.putNumber("ury", ury);
    // SmartDashboard.putNumber("centerX", centerX);


  }

  /**
   * This function is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {
    //new NetworkTableRun().run();
    uly = new NetworkTableRun().getTableNum("ul.y");
    ury = new NetworkTableRun().getTableNum("ur.y");
    centerX = new NetworkTableRun().getTableNum("centerX");
    distance = new NetworkTableRun().getTableNum("distance(cm)");
    
    SmartDashboard.putNumber("uly", uly);
    SmartDashboard.putNumber("ury", ury);
    SmartDashboard.putNumber("centerX", centerX);
    SmartDashboard.putNumber("distance(cm)", distance);
    
    if(m_stick.getRawAxis(4) > 0.3 || m_stick.getRawAxis(4) < -0.3 || m_stick.getRawAxis(5) > 0.3 || m_stick.getRawAxis(5) < -0.3)
    {
    m_robotDrive.arcadeDrive(m_stick.getRawAxis(5) * 0.6 * -1, m_stick.getRawAxis(4)  * 0.6);
    }
    else if(m_stick.getRawButton(1)){
      errA = (uly - ury) * - 0.002;
      errC = (centerX - 159.5) * 0.002;
      // if(err > 1){
      //   err = 1;
      // }else if(err < -1){
      //   err = -1;
      // }
      m_robotDrive.arcadeDrive(0.5 * errC, 0.5 * errA);
    }
    else
    {
    m_robotDrive.stopMotor();
    }

    
    // System.out.println("Right Pos" +  ToDeg(rightPos));
    // System.out.println("Left Pos" +  ToDeg(leftPos));
    // System.out.println("Error" +  err);
    // SmartDashboard.putNumber("Right Pos", ToDeg(rightPos));
    // SmartDashboard.putNumber("Left Pos", ToDeg(leftPos));
    // SmartDashboard.putNumber("Error", err);




  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  // 	/**
	//  * @param units CTRE mag encoder sensor units 
	//  * @return degrees rounded to tenths.
	//  */
	// double ToDeg(final int units) {
	// 	double deg = units * 360.0 / 4096.0;

	// 	/* truncate to 0.1 res */
	// 	deg *= 10;
	// 	deg = (int) deg;
	// 	deg /= 10;

	// 	return deg;
  // }
  
}
