// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


import javax.swing.AbstractCellEditor;
import javax.swing.plaf.nimbus.AbstractRegionPainter;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.revrobotics.CANSparkMax; //imports the libary for Spark Max control over CAN
import com.revrobotics.CANSparkMaxLowLevel; //additional functionality for Spark motor controllers
import com.revrobotics.CANSparkMaxLowLevel.MotorType; //allows for brushless motor control
import com.revrobotics.RelativeEncoder; //allows the NEO ecoders to be used
import com.revrobotics.SparkMaxPIDController; //allows for motor controller PID control
import edu.wpi.first.wpilibj.DriverStation;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
 
private MecanumDrive robotDrive;
private Joystick driverStick;
private Joystick operatorStick;
double turnRate;
DoubleSolenoid wristSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0,1);
DoubleSolenoid brakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2,3);
DoubleSolenoid gripSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4,5);

WPI_TalonFX Frontleft = new WPI_TalonFX(12,"rio");
WPI_TalonFX Rearleft = new WPI_TalonFX(11,"rio");
WPI_TalonFX Frontright = new WPI_TalonFX(13,"rio");
WPI_TalonFX Rearright = new WPI_TalonFX(14,"rio");

final CANSparkMax armMotor = new CANSparkMax(16, MotorType.kBrushless); //creates and names a motor controller CAN ID 6 and makes it brushless
final CANSparkMax extensionMotor = new CANSparkMax(15, MotorType.kBrushless); //creates and names a motor controller CAN ID 6 and makes it brushless

private RelativeEncoder armEncoder;
private RelativeEncoder extensionEncoder;
  DigitalInput armCalSwitch = new DigitalInput(1);
  DigitalInput extensionCalSwitch = new DigitalInput(0);
  public boolean armCalState; //stores the state of arm caliberation
  public boolean extensionCalState; //stores the state of arm caliberation

  private SparkMaxPIDController extensionPID; //creates a PID controller for the extension
  private SparkMaxPIDController armPID; //creates a PID controller for the arm

@Override
  public void robotInit() {
   

    driverStick = new Joystick(0);
    operatorStick = new Joystick(1);

    robotDrive = new MecanumDrive(Frontleft, Rearleft, Frontright, Rearright);
    Frontleft.setInverted(true);
    Rearleft.setInverted(true);

   
  extensionEncoder = extensionMotor.getEncoder(); //attaches the extension encoder to the extensionMotor 
  extensionPID = extensionMotor.getPIDController(); //attaches arm PID settings to the armMotor controller
   
    extensionPID.setP(.1);
    extensionPID.setI(0);
    extensionPID.setD(0);
    extensionPID.setIZone(0);
    extensionPID.setFF(0);
    extensionPID.setOutputRange(-1, 1);

 armEncoder = armMotor.getEncoder(); //attaches the arm encoder to the armMotor motor
 armPID = armMotor.getPIDController(); //attaches arm PID settings to the armMotor controller
   
    armPID.setP(.1);
    armPID.setI(0);
    armPID.setD(0);
    armPID.setIZone(0);
    armPID.setFF(0);
    //can lift at full power but limits drop power
    armPID.setOutputRange(-.5, .15);
  }


  @Override
  public void robotPeriodic() {
    //set up network table for limelight
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry tv = table.getEntry("tv");
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double targetAquired = tv.getDouble(0.0);

    //get wheel encoders
    double frontLeftVal = Frontleft.getSelectedSensorVelocity(0); /* position units per 100ms */
    double rearLeftVal = Rearleft.getSelectedSensorVelocity(0);
    double frontRightVal = Frontright.getSelectedSensorVelocity(0);
    double rearRightVal = Rearright.getSelectedSensorVelocity(0);

    //smart dashboard display
    SmartDashboard.putNumber("Front Left Encoder", frontLeftVal); // displays the velocity of the wheels
    SmartDashboard.putNumber("Rear Left Encoder", rearLeftVal);
    SmartDashboard.putNumber("Front Right Encoder", frontRightVal);
    SmartDashboard.putNumber("Rear Right Encoder", rearRightVal);
    SmartDashboard.putBoolean("Extension Switch Output", extensionCalSwitch.get()); //displays the real time state of the extension calibration swtich
    SmartDashboard.putBoolean("Extension Calibration State", extensionCalState); //displays the real time state of the extension calibration swtich
    SmartDashboard.putNumber(" Extension Position", extensionEncoder.getPosition()); //puts the postion on the extension motor (in revolutions) on the dashboard
    SmartDashboard.putBoolean("Arm Switch Output", armCalSwitch.get()); //displays the real time state of the arm calibration swtich
    SmartDashboard.putBoolean("Arm Calibration State", armCalState); //displays the real time state of the arm calibration swtich
    SmartDashboard.putNumber("Arm Position", armEncoder.getPosition()); //puts the postion on the arm motor (in revolutions) on the dashboard
  }

  @Override
  public void autonomousInit() {
    //set limelight to vision processing mode
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);

    if(DriverStation.getAlliance()==Alliance.Blue){
      //set pipeline to 7 for apriltag 7
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(7);
     }
     else{ 
      //set pipeline to 2 for apriltag 2
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(2);
    }
    armEncoder.setPosition((double) 0);
    extensionEncoder.setPosition((double)0);
  }

  @Override
  public void autonomousPeriodic() {
    
  }

  @Override
  public void teleopInit() {
    //sets camera mode to regular camera
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
  }

  @Override
  public void teleopPeriodic() {
    //driver Commands
    if(driverStick.getRawButton(4)){
      turnRate=-.2;
    }
    else if(driverStick.getRawButton(5)){
      turnRate=.2;
    }
    else{turnRate=0;
    }
    robotDrive.driveCartesian(driverStick.getY(), -driverStick.getX(), -turnRate);
    if(driverStick.getRawButtonPressed(6)){
      brakeSolenoid.set(Value.kForward);
    }
    if(driverStick.getRawButtonReleased(6)){
      brakeSolenoid.set(Value.kReverse);
    }
    

//operator commands
//player station
if(operatorStick.getRawButton(1)){
  wristSolenoid.set(Value.kReverse);
  gripSolenoid.set(Value.kReverse);
  armPID.setReference(-75,CANSparkMax.ControlType.kPosition);
  extensionPID.setReference(-20, CANSparkMax.ControlType.kPosition);
}
//high goal
if(operatorStick.getRawButton(5)){
  wristSolenoid.set(Value.kForward);
  armPID.setReference(-75,CANSparkMax.ControlType.kPosition);
  extensionPID.setReference(-250, CANSparkMax.ControlType.kPosition);
}
//set medium goal
if(operatorStick.getRawButton(3)){
  wristSolenoid.set(Value.kForward);
  armPID.setReference(-62,CANSparkMax.ControlType.kPosition);
  extensionPID.setReference(-138, CANSparkMax.ControlType.kPosition);
}
//set low goal
if(operatorStick.getRawButton(4)){
  wristSolenoid.set(Value.kForward);
  armPID.setReference(-23,CANSparkMax.ControlType.kPosition);
  extensionPID.setReference(-154, CANSparkMax.ControlType.kPosition);
}

//set seeking position
if(operatorStick.getRawButton(2)){
  armPID.setReference(-15,CANSparkMax.ControlType.kPosition);
  extensionPID.setReference(-240, CANSparkMax.ControlType.kPosition);
  wristSolenoid.set(Value.kForward);
  gripSolenoid.set(Value.kReverse);
}
if(operatorStick.getRawButton(6)){
  gripSolenoid.set(Value.kForward);
}
if(operatorStick.getRawButton(7)){
  gripSolenoid.set(Value.kReverse);
}

  }


  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {
    armCalState = false;
    extensionCalState = false;
  }

  @Override
  public void testPeriodic() {
    wristSolenoid.set(Value.kForward);
    brakeSolenoid.set(Value.kForward);
    gripSolenoid.set(Value.kForward);
    while(extensionCalState==false){ //if the extension is not caliberated
      extensionMotor.set(.2); //run the extension motor into the caliberation switch
      if(extensionCalSwitch.get()==true){ //when the extension switch is pressed
        extensionCalState=true; //set the extension calibiration state to true
        extensionMotor.set(0); //stop the extension motor
        extensionEncoder.setPosition((double) 0); //zero the encoder
      }
    }
    while(armCalState==false){ //if the arm is not caliberated
      armMotor.set(.1); //run the arm motor into the caliberation switch
      if(armCalSwitch.get()==true){ //when the arm switch is pressed
        armCalState=true; //set the arm calibiration state to true
        armMotor.set(0); //stop the arm motor
        armEncoder.setPosition((double) 0); //zero the encoder
      }
    }
    
  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}