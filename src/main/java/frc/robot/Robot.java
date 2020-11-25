/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
/*import ctr-e libraries*/
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.platform.can.AutocacheState;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */

  // PHASE 1 START
  public WPI_TalonFX rightMain = new WPI_TalonFX(1);
  public WPI_TalonFX rightFollow = new WPI_TalonFX(2);
  public WPI_TalonFX leftMain = new WPI_TalonFX(3);
  public WPI_TalonFX leftFollow = new WPI_TalonFX(4);

  public DifferentialDrive twoMotorDrive = new DifferentialDrive(rightMain, leftMain);

  public Joystick leftStick;
  public Joystick rightStick;
  // PHASE 1 END

  // PHASE 2 START
  boolean leftTrigger = leftStick.getRawButton(0);
  boolean leftThumbMain = leftStick.getRawButton(1);
  boolean leftThumbLeft = leftStick.getRawButton(2);
  boolean leftThumbRight = leftStick.getRawButton(3);
  boolean leftRightArrayTR = leftStick.getRawButton(4);
  boolean leftRightArrayTM = leftStick.getRawButton(5);
  boolean leftRightArrayTL = leftStick.getRawButton(6);
  boolean leftRightArrayBL = leftStick.getRawButton(7);
  boolean leftRightArrayBM = leftStick.getRawButton(8);
  boolean leftRightArrayBR = leftStick.getRawButton(9);
  boolean leftLeftArrayTL = leftStick.getRawButton(10);
  boolean leftLeftArrayTM = leftStick.getRawButton(11);
  boolean leftLeftArrayTR = leftStick.getRawButton(12);
  boolean leftLeftArrayBR = leftStick.getRawButton(13);
  boolean leftLeftArrayBM = leftStick.getRawButton(14);
  boolean leftLeftArrayBL = leftStick.getRawButton(15);

  //Right joystick buttons:
  boolean rightTrigger = rightStick.getRawButton(0);
  boolean rightThumbMain = rightStick.getRawButton(1);
  boolean rightThumbLeft = rightStick.getRawButton(2);
  boolean rightThumbRight = rightStick.getRawButton(3);
  boolean rightLeftArrayTL = rightStick.getRawButton(4);
  boolean rightLeftArrayTM = rightStick.getRawButton(5);
  boolean rightLeftArrayTR = rightStick.getRawButton(6);
  boolean rightLeftArrayBR = rightStick.getRawButton(7);
  boolean rightLeftArrayBM = rightStick.getRawButton(8);
  boolean rightLeftArrayBL = rightStick.getRawButton(9);
  boolean rightRightArrayTR = rightStick.getRawButton(10);
  boolean rightRightArrayTM = rightStick.getRawButton(11);
  boolean rightRightArrayTL = rightStick.getRawButton(12);
  boolean rightRightArrayBL = rightStick.getRawButton(13);
  boolean rightRightArrayBM = rightStick.getRawButton(14);
  boolean rightRightArrayBR = rightStick.getRawButton(15);
  // PHASE 2 END

  // PHASE 3 START
  boolean shooterSideIsForward = false;
  // PHASE 3 END

  // PHASE 4 START
  boolean climbSolenoidToggle = false;
  Solenoid climberPiston = new Solenoid(0,0);
  WPI_TalonFX climberTalon = new WPI_TalonFX(11);
  WPI_TalonFX climberTalonFollow = new WPI_TalonFX(12);
  // PHASE 4 END

  // PHASE 5 START
  TimeOfFlight ToF1 = new TimeOfFlight(1);
  TimeOfFlight ToF2 = new TimeOfFlight(2);
  TimeOfFlight ToF3 = new TimeOfFlight(3);
  TimeOfFlight ToF4 = new TimeOfFlight(4);
  // PHASE 5 END

  // PHASE 6 START
  WPI_TalonFX shooter = new WPI_TalonFX(5);
  WPI_TalonFX shooterFollow = new WPI_TalonFX(6);

  double leftSlider = leftStick.getRawAxis(0);

  boolean fireStatus = false;
  // PHASE 6 END

  // PHASE 8 START 
  WPI_TalonFX accelerator = new WPI_TalonFX(7);
  // PHASE 8 END

  // PHASE 9 START
  WPI_TalonFX innerIntake = new WPI_TalonFX(9);
  // PHASE 9 END
  
  // PHASE 10 START 
  WPI_TalonFX conveyor = new WPI_TalonFX(8);
  // PHASE 10 END

  @Override
  public void robotInit() {
    // PHASE 1 START
    rightMain.configFactoryDefault();
    rightFollow.configFactoryDefault();
    leftMain.configFactoryDefault();
    leftFollow.configFactoryDefault();

    rightFollow.follow(rightMain);
    leftFollow.follow(leftMain);
    // PHASE 1 END

    // PHASE 4 START
    climberTalonFollow.setInverted(true);
    climberTalonFollow.follow(climberTalon);

    climberTalon.enableVoltageCompensation(true);
    climberTalonFollow.enableVoltageCompensation(true);

    climberTalon.configVoltageCompSaturation(12);
    climberTalonFollow.configVoltageCompSaturation(12);
    // PHASE 4 END

    // PHASE 5 START
    ToF1.setRangingMode(RangingMode.Short,24);
    ToF2.setRangingMode(RangingMode.Short,24);
    ToF3.setRangingMode(RangingMode.Short,24);
    ToF4.setRangingMode(RangingMode.Short,24);
    // PHASE 5 END

    // PHASE 6 START
    shooterFollow.setInverted(true);
    shooterFollow.follow(shooter);
    // Shooter config pidf which I have no idea how to do
    // PHASE 6 END

    // PHASE 8 START
    accelerator.enableVoltageCompensation(true);
    accelerator.configVoltageCompSaturation(12);
    // PHASE 8 END

    // PHASE 9 START
    innerIntake.enableVoltageCompensation(true);
    innerIntake.configVoltageCompSaturation(12);
    // PHASE 9 END

    // PHASE 11 START
    conveyor.enableVoltageCompensation(true);
    conveyor.configVoltageCompSaturation(12);
    // PHASE 11 END
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    // PHASE 1 START
    twoMotorDrive.tankDrive(leftStick.getY(), rightStick.getY());
    // PHASE 1 END

    // PHASE 2 and 3 START
    if (rightRightArrayBM) {
      // Limelight code
    } else {
      // Network table stuff

      if (shooterSideIsForward) {
        twoMotorDrive.tankDrive(-leftStick.getY(), rightStick.getY());
      } else {
        twoMotorDrive.tankDrive(leftStick.getY(), -rightStick.getY());
      }
    }

    if (rightThumbLeft) {
      shooterSideIsForward = false;
    } else {
      if (rightThumbRight) {
        shooterSideIsForward = true;
      } else {

      }
    }
    // PHASE 2 and 3 END

    // PHASE 4 START
    if (climbSolenoidToggle) {
      climberPiston.set(true);
    } else {
      climberPiston.set(false);
    }

    if (rightRightArrayTL) {
      climberTalon.set(ControlMode.PercentOutput, .7);
    } else {
      climberTalon.set(ControlMode.PercentOutput, 0);
    }

    if (rightThumbMain) {
      if (climbSolenoidToggle) {
        climbSolenoidToggle = false;
      } else {
        climbSolenoidToggle = true;
      }
    }
    // PHASE 4 END

    // PHASE 7 START 
    double shooterSpeed = getShooterSpeed();

    shooter.set(shooterSpeed);

    if ((shooter.getSelectedSensorVelocity() < 1.5*shooterSpeed) && (shooter.getSelectedSensorVelocity() > .95*(shooterSpeed))) {
      fireStatus = true;
    } else {
      fireStatus = false;
    }
    // PHASE 7 END

    // PHASE 8, PHASE 9, PHASE 10, PHASE 11, PHASE 12, PHASE 13 START
    if (leftTrigger) {
      accelerator.set(1);
      manualUpdateConveyorSpeed();
      manualUpdateInnerIntakeSpeed();
    } else {
      accelerator.set(0);
      autoUpdateConveyorSpeed();
      autoUpdateInnerIntakeSpeed();
    }
    // PHASE 8, PHASE 9, PHASE 10, PHASE 11, PHASE 12, PHASE 13 END

  }

  // PHASE 5 START
  public boolean ToF1SeesBall() {
    if (ToF1.getRange() < 110) {
      return true;
    } else {
      return false;
    }
  }

  public boolean ToF2SeesBall() {
    if (ToF2.getRange() < 100) {
      return true;
    } else {
      return false;
    }
  }

  public boolean ToF3SeesBall() {
    if (ToF3.getRange() < 100) {
      return true;
    } else {
      return false;
    }
  }

  public boolean ToF4SeesBall() {
    if (ToF4.getRange() < 110) {
      return true;
    } else {
      return false;
    }
  }
  // PHASE 5 END

  // PHASE 6 START
  public double getShooterSpeed() {
    double manualShooterSpeed = 16500;
    double autoShooterSpeed = 0;

    double leftSliderAddition = ((-leftSlider+1)/2)*4000;

    if (leftThumbRight) {
      return manualShooterSpeed + leftSliderAddition;
    } else {
      if (leftThumbMain) {
        return autoShooterSpeed + leftSliderAddition;
      } else {
        return leftSliderAddition;
      }
    }
  }
  // PHASE 6 END

  // PHASE 9 START
  public void autoUpdateInnerIntakeSpeed() {
    if(rightTrigger && (!ToF1SeesBall()&&!ToF4SeesBall())) {
      if (leftLeftArrayBL) {
        innerIntake.set(1);
      } else {
        innerIntake.set(-1);
      }
    } else {
      innerIntake.set(0);
    }
  }
  // PHASE 9 END

  // PHASE 10 START
  public void manualUpdateInnerIntakeSpeed() {
    innerIntake.set(.4);
  }
  // PHASE 10 END

  // PHASE 11 START
  public void autoUpdateConveyorSpeed() {
    boolean ToFBoolean = (!ToF4SeesBall() && (ToF2SeesBall() || ToF3SeesBall()));

    if (ToFBoolean) {
      if (leftLeftArrayBL) {
        conveyor.set(-1);
      } else {
        conveyor.set(.6);
      }
    } else {
      conveyor.set(0);
    }
  }
  // PHASE 11 END

  // PHASE 12 START
  public void manualUpdateConveyorSpeed() {
    conveyor.set(.6);
  }
  // PHASE 12 END

  // PHASE 13 START

  // PHASE 13 END
}
