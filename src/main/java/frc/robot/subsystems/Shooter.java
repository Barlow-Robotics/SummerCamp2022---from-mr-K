// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.sim.PhysicsSim;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Shooter extends SubsystemBase {
    // Creates a new Shooter

    WPI_TalonFX m_flywheelMotor;

    boolean isShooting = false;
    boolean simulationInitialized = false;

    DigitalInput blueBot;


    public Shooter() {
        m_flywheelMotor = new WPI_TalonFX(Constants.ShooterConstants.Flywheel.ID_Motor);
        blueBot = new DigitalInput(9) ;

        setMotorConfig(m_flywheelMotor);
        if (blueBot.get()) {
            m_flywheelMotor.setInverted(TalonFXInvertType.CounterClockwise);
        } else {
            m_flywheelMotor.setInverted(TalonFXInvertType.Clockwise);
        }
    }

    public void startShooting() {
        m_flywheelMotor.set(TalonFXControlMode.Velocity, Constants.ShooterConstants.Flywheel.FlywheelVelocity);
        isShooting = true;
    }

    public void stopShooting() {
        m_flywheelMotor.set(TalonFXControlMode.Velocity, 0);
        isShooting = false;
    }

    public double getFlywheelSpeed() {
        double s = m_flywheelMotor.getSelectedSensorVelocity() ;
        return (s);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    private void setMotorConfig(WPI_TalonFX motor) {
        motor.configFactoryDefault();
        motor.configOpenloopRamp(0.1);
        motor.configClosedloopRamp(Constants.ShooterConstants.closedVoltageRampingConstant);
        motor.configOpenloopRamp(Constants.ShooterConstants.manualVoltageRampingConstant);
        motor.config_kF(Constants.ShooterConstants.Flywheel.PID_id, Constants.ShooterConstants.Flywheel.kF);
        motor.config_kP(Constants.ShooterConstants.Flywheel.PID_id, Constants.ShooterConstants.Flywheel.kP);
        motor.config_kI(Constants.ShooterConstants.Flywheel.PID_id, Constants.ShooterConstants.Flywheel.kI);
        motor.config_kD(Constants.ShooterConstants.Flywheel.PID_id, Constants.ShooterConstants.Flywheel.kD);
        motor.setNeutralMode(NeutralMode.Coast);
    }

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(m_flywheelMotor, 0.5, 6800);
        // PhysicsSim.getInstance().addTalonSRX(m_hoodMotor, 0.5, 6800);
    }

    @Override
    public void simulationPeriodic() {
        if (!simulationInitialized) {
            simulationInit();
            simulationInitialized = true;
        }
    }
}
