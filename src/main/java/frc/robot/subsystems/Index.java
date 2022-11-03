// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.sim.PhysicsSim;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Index extends SubsystemBase {

  WPI_TalonSRX m_hopperMotor;
  WPI_TalonSRX m_feederMotor;

  boolean feederIsRunning = false;  
  boolean simulationInitialized = false;

  /** Creates a new Index. */
  public Index() {
    m_hopperMotor = new WPI_TalonSRX(Constants.IndexConstants.ID_HopperMotor);
    m_feederMotor = new WPI_TalonSRX(Constants.IndexConstants.ID_FeederMotor);
  
    setHopperMotorConfig(m_hopperMotor);
    setFeederMotorConfig(m_feederMotor);

    //m_hopperMotor.setInverted(InvertType.InvertMotorOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void startFeeder() {
    m_feederMotor.set(TalonSRXControlMode.PercentOutput, Constants.IndexConstants.feederMotorSpeed);
    feederIsRunning = true;
  }

  public void stopFeeder() {
    m_feederMotor.set(TalonSRXControlMode.PercentOutput, 0);
    feederIsRunning = false;
  }
  
  public void startHopper() {
    m_hopperMotor.set(TalonSRXControlMode.PercentOutput, Constants.IndexConstants.hopperMotorSpeed);
  }

  public void stopHopper() {
      m_hopperMotor.set(TalonSRXControlMode.PercentOutput, 0);
  }

  private void setHopperMotorConfig(WPI_TalonSRX motor) {
    motor.configFactoryDefault();
    motor.configClosedloopRamp(Constants.IndexConstants.closedVoltageRampingConstant);
    motor.configOpenloopRamp(Constants.IndexConstants.manualVoltageRampingConstant);
    motor.config_kF(Constants.IndexConstants.PID_id, Constants.IndexConstants.Hopper_kF);
    motor.config_kP(Constants.IndexConstants.PID_id, Constants.IndexConstants.Hopper_kP);
    motor.config_kI(Constants.IndexConstants.PID_id, Constants.IndexConstants.Hopper_kI);
    motor.config_kD(Constants.IndexConstants.PID_id, Constants.IndexConstants.Hopper_kD);
    motor.setNeutralMode(NeutralMode.Brake);
  }

  private void setFeederMotorConfig(WPI_TalonSRX motor) {
    motor.configFactoryDefault();
    motor.configClosedloopRamp(Constants.IndexConstants.closedVoltageRampingConstant);
    motor.configOpenloopRamp(Constants.IndexConstants.manualVoltageRampingConstant);
    motor.config_kF(Constants.IndexConstants.PID_id, Constants.IndexConstants.Feeder_kF);
    motor.config_kP(Constants.IndexConstants.PID_id, Constants.IndexConstants.Feeder_kP);
    motor.config_kI(Constants.IndexConstants.PID_id, Constants.IndexConstants.Feeder_kI);
    motor.config_kD(Constants.IndexConstants.PID_id, Constants.IndexConstants.Feeder_kD);
    motor.setNeutralMode(NeutralMode.Brake);
  }

  public void simulationInit() {
      PhysicsSim.getInstance().addTalonSRX(m_feederMotor, 0.5, 6800);
      PhysicsSim.getInstance().addTalonSRX(m_hopperMotor, 0.5, 6800);
    }

  @Override
  public void simulationPeriodic() {
      if (!simulationInitialized) {
          simulationInit();
          simulationInitialized = true;
      }
  }
}
