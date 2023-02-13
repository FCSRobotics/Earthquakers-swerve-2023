// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.looper.Looper;
import frc.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.PowerDistribution;



public class Robot extends TimedRobot {

  public static PowerDistribution pdp;
  public static Drivetrain drivetrain;
  // public static BarIndexer indexer;
  // public static Suction suction;

  public static Compressor compressor;
  
  private Joystick leftStick;
  private Joystick rightStick;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    drivetrain = Drivetrain.getDriveInstance(pdp);


    addPeriodic(() -> {
      drivetrain.update();
    }, .01, 0.005);

   
    leftStick = new Joystick(0);
    rightStick = new Joystick(1);
    drivetrain.resetGyro();
    drivetrain.outputOdometry();
    //drivetrain.resetEncoder();

  }
  @Override
  public void robotPeriodic(){
    drivetrain.outputOdometry(); 
  }

  @Override
  public void autonomousInit() {
  }

  public void autonomousPeriodic() {
  }


  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {

      drivetrain.inputHandler((Math.abs(leftStick.getY()) < 0.01f ? 0.0f : leftStick.getY() ) /2, (Math.abs(leftStick.getX()) < 0.01f ? 0.0f : leftStick.getX() ) /2, (Math.abs(rightStick.getX()) < 0.05f ? 0.0f : rightStick.getX() ) / 1.5);   
      SmartDashboard.putNumber("Joystick X:", leftStick.getX()); 
  }

  @Override
  public void disabledInit() {
    drivetrain.stop();  
  }

  @Override
  public void disabledPeriodic() {

  }

  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  public static Drivetrain getDrivetrainInstance(){
    return drivetrain;
  }
}
