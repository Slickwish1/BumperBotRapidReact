// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  VictorSP leftMotorFront = new VictorSP(5);
  VictorSP leftMotorBack = new VictorSP(6);

  VictorSP rightMotorFront = new VictorSP(0);
  VictorSP rightMotorBack = new VictorSP(1);

  MotorControllerGroup rightMotors = new MotorControllerGroup(rightMotorFront, rightMotorBack);
  MotorControllerGroup leftMotors = new MotorControllerGroup(leftMotorFront, leftMotorBack);

  

  private final DifferentialDrive m_robotDrive = new DifferentialDrive(leftMotors, rightMotors);

  public DriveSystem() {
    leftMotors.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void drive(double d, double e) {
    m_robotDrive.arcadeDrive(-d, e, false);
  }
}
