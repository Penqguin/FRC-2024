// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {

  private final WPI_TalonSRX leftfront = new WPI_TalonSRX(1);
  private final WPI_TalonSRX leftrear = new WPI_TalonSRX(2);

  private final WPI_TalonSRX rightfront = new WPI_TalonSRX(3);
  private final WPI_TalonSRX rightrear = new WPI_TalonSRX(4);

  //hypothetical arm stuff

  private final WPI_TalonSRX rightarm = new WPI_TalonSRX(5);
  private final WPI_TalonSRX leftarm = new WPI_TalonSRX(6);
  
  private final DifferentialDrive robotDrive = new DifferentialDrive(leftfront, rightfront);

  private final XboxController controller = new XboxController(0);

  private double start;

  public Robot(){
    SendableRegistry.addChild(robotDrive, leftfront);
    SendableRegistry.addChild(robotDrive, rightfront);
  }

  @Override
  public void robotInit() {
    leftfront.setInverted(true);
    leftrear.setInverted(true);
    leftarm.setInverted(true);

    leftrear.follow(leftfront);
    rightrear.follow(rightfront);
    leftarm.follow(rightarm);
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
    start = Timer.getFPGATimestamp();
  }

  @Override
  public void autonomousPeriodic() {

    double autotimer = Timer.getFPGATimestamp();

    //increase or decrease the two to set the amount of time robot drives forward
    if(autotimer-start < 2){
    leftfront.set(0.7);
    rightfront.set(0.7);
    }

    //stop motors after auto ends
    else{
      Stopdrive();
    }
  }


  @Override
  public void teleopInit() {
  }

  //stop drivemotors function
  public void Stopdrive(){
    leftfront.set(0);
    rightfront.set(0);
  }

  //stop armmotors function
  public void Stoparm(){
    rightarm.set(0);
  }

  //drivetrain method
  public void Drivecode(double Leftjoy, double Rightjoy){
    if (Leftjoy > 0.1 || Leftjoy < -0.1) {
      leftfront.set(Leftjoy);
      rightfront.set(Leftjoy);
    } 
    else if (Rightjoy > 0.1) {
      leftfront.set(Rightjoy);
      rightfront.set(-Rightjoy / 1.5);
    } 
    else if (Rightjoy < -0.1) {
      leftfront.set(-Rightjoy / 1.5);
      rightfront.set(Rightjoy);
    } 
    else {
      Stopdrive();
    }
  }

  //arm method
    public void Armcode(double Yjoy){
    if (Yjoy > 0.1 || Yjoy < -0.1) {
      rightarm.set(Yjoy/2);
      
      //add a check to make sure arm stays in it's range 
    } 
    else {
      Stoparm();
    }
  } 

  @Override
  public void teleopPeriodic() {
    //joystick inputs
    double Ljoystick = controller.getLeftY();
    double Rjoystick = controller.getRightX();
    double armstick = controller.getRightY();

    //the drive method
    Drivecode(Ljoystick, Rjoystick);
    //the arm method
    Armcode(armstick);
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
