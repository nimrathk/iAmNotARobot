// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

// import edu.wpi.first.wpilibj.Compressor;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkMaxRelativeEncoder;

public class Robot extends TimedRobot {
  private final WPI_VictorSPX m_leftFrontMotor = new WPI_VictorSPX(6);
  private final WPI_VictorSPX m_rightFrontMotor = new WPI_VictorSPX(5);
  private final WPI_VictorSPX m_leftRearMotor = new WPI_VictorSPX(3);
  private final WPI_VictorSPX m_rightRearMotor = new WPI_VictorSPX(2);

  private final XboxController m_stickDrive = new XboxController(0);
  private final XboxController m_stickArm = new XboxController(1);

  private final MotorControllerGroup leftMotors = new MotorControllerGroup(m_leftFrontMotor, m_leftRearMotor);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(m_rightFrontMotor, m_rightRearMotor);
  private DifferentialDrive diffDrive = new DifferentialDrive(leftMotors, rightMotors);

  private static final String kDriveAuto = "Drive";
  private static final String kRandAuto = "Random Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final Timer m_timer = new Timer();

  // private final Compressor theCompressor = new Compressor(PneumaticsModuleType.CTREPCM);
  // Compressor compressor;

  private CANSparkMax mPivotMotor;
  private CANSparkMax mElevatorMotor;
  private CANSparkMax mGripperMotor;
  // private RelativeEncoder mPivotEncoder;
  private SparkMaxPIDController mPivotPIDController;
  private SparkMaxPIDController mElevatorPIDController;
  private SparkMaxPIDController mGripperPIDController;

  private static final double kPivotPowerOut = 1.0;
  private static final double kPivotPowerIn = -0.7;
  
  private static final double kElevatorPowerOut = 1.0;
  private static final double kElevatorPowerIn = -0.7;

  private static final double kGripperPowerOut = 1.0;
  private static final double kGripperPowerIn = -1.0;

  // private static final double kExtensionPowerOut = 0.6;
  // private static final double kExtensionPowerIn = -0.6;
  // private static final double kPivotBoostAmount = -3;
  // private static final double kPivotBoost2Amount = -15;

  private static final double kPivotCLRampRate = 0.5;
  private static final double kElevatorCLRampRate = 0.5;
  private static final double kGripperCLRampRate = 0.5;
  // private static final double kExtensionCLRampRate = 0.5;

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightFrontMotor.setInverted(true);
    m_rightRearMotor.setInverted(true);

    diffDrive.setDeadband(0.04);

    // PIVOT Stuff
    mPivotMotor = new CANSparkMax(4, MotorType.kBrushless);
    mPivotMotor.restoreFactoryDefaults();
    mPivotPIDController = mPivotMotor.getPIDController();
      //mPivotEncoder = mPivotMotor.getEncoder();

    mPivotPIDController.setP(0.1);
    mPivotPIDController.setI(1e-8);
    mPivotPIDController.setD(1);
    mPivotPIDController.setIZone(0);
    mPivotPIDController.setFF(0);
    mPivotPIDController.setOutputRange(kPivotPowerIn, kPivotPowerOut);
    mPivotMotor.setClosedLoopRampRate(kPivotCLRampRate);

    mPivotPIDController.setReference(0, CANSparkMax.ControlType.kPosition);
    
    // ELEVATOR stuff
    mElevatorMotor = new CANSparkMax(9, MotorType.kBrushless);
    mElevatorMotor.restoreFactoryDefaults();
    mElevatorPIDController = mElevatorMotor.getPIDController();

    mElevatorPIDController.setP(0.1);
    mElevatorPIDController.setI(1e-8);
    mElevatorPIDController.setD(1);
    mElevatorPIDController.setIZone(0);
    mElevatorPIDController.setFF(0);
    mElevatorPIDController.setOutputRange(kElevatorPowerIn, kElevatorPowerOut);
    mElevatorMotor.setClosedLoopRampRate(kElevatorCLRampRate);

    mElevatorPIDController.setReference(0, CANSparkMax.ControlType.kPosition);

    // GRIPPER stuff
    mGripperMotor = new CANSparkMax(7, MotorType.kBrushless);
    mGripperMotor.restoreFactoryDefaults();
    mGripperPIDController = mGripperMotor.getPIDController();

    mGripperPIDController.setP(6e-5);
    mGripperPIDController.setI(0);
    mGripperPIDController.setD(0);
    mGripperPIDController.setIZone(0);
    mGripperPIDController.setFF(0.00015);
    mGripperPIDController.setOutputRange(kGripperPowerIn, kGripperPowerOut);
    mGripperMotor.setClosedLoopRampRate(kGripperCLRampRate);

    mGripperPIDController.setReference(0, CANSparkMax.ControlType.kPosition);
  }
  //matthewgpage@gmail.com
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();

    m_chooser.setDefaultOption("Drive", kDriveAuto);
    m_chooser.addOption("Random Auto", kRandAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
  }

  @Override
  public void autonomousPeriodic() {
    double elapsed = m_timer.get();
    
    switch (m_autoSelected) {
      case kRandAuto:
        // Put custom auto code here
        break;
      case kDriveAuto:
      default:
        // Put default auto code here
        while(elapsed < 10)
        {
          diffDrive.arcadeDrive(0.25, 0);
        }
        break;
    }
  }

  @Override
  public void teleopPeriodic() {
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.

    if(Math.abs(m_stickDrive.getLeftY()) > 0.2 || Math.abs(m_stickDrive.getLeftX()) > 0.2)
    {
      diffDrive.arcadeDrive(-m_stickDrive.getLeftY() * 0.75, -m_stickDrive.getLeftX() * 0.75);
    }
    else
    {
      diffDrive.arcadeDrive(-m_stickDrive.getRightY(), (-m_stickDrive.getRightX()) * 0.75);
    }

    if(m_stickArm.getBButtonPressed())
    {
      mPivotPIDController.setReference(30, CANSparkMax.ControlType.kPosition);
    }
    else if(m_stickArm.getAButtonPressed())
    {
      mPivotPIDController.setReference(0, CANSparkMax.ControlType.kPosition);
    }

    if(m_stickArm.getYButtonPressed())
    {
      mElevatorPIDController.setReference(50, CANSparkMax.ControlType.kPosition);
    }
    if(m_stickArm.getXButtonPressed())
    {
      mElevatorPIDController.setReference(0, CANSparkMax.ControlType.kPosition);
    }

    if(m_stickArm.getLeftTriggerAxis() > 0)
    {
      mGripperPIDController.setReference(3000, CANSparkMax.ControlType.kVelocity);
    }
    else if(m_stickArm.getRightTriggerAxis() > 0)
    {
      mGripperPIDController.setReference(-3000, CANSparkMax.ControlType.kVelocity);
    }
    else 
    {
      mGripperPIDController.setReference(0, CANSparkMax.ControlType.kVelocity);
    }
  }
}
