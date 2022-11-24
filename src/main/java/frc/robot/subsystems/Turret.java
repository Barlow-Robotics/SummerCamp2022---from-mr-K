// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.Constants;
import frc.robot.sim.PhysicsSim;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;

public class Turret extends SubsystemBase {

    WPI_TalonSRX m_turretMotor;

    DigitalInput leftLimitSwitch;
    DigitalInput rightLimitSwitch;

    /** Creates a new Turret. */
    public Turret() {
        m_turretMotor = new WPI_TalonSRX(Constants.ShooterConstants.Turret.ID_Motor);
        setMotorConfig(m_turretMotor);
        m_turretMotor.setInverted(InvertType.InvertMotorOutput);

        leftLimitSwitch = new DigitalInput(Constants.ShooterConstants.Turret.ID_LeftLimitSwitch);
        rightLimitSwitch = new DigitalInput(Constants.ShooterConstants.Turret.ID_RightLimitSwitch);
    }

    public void rotateTurret(double rotateVelocity) {
        // if the limit switch is hit...

        // double output = 0.0;
        double output = rotateVelocity;

        if (rotateVelocity > 0.0 && leftLimitSwitch.get()) {
            // output = rotateVelocity;
            output = 0.0;
        }
        //  else if (rotateVelocity < 0.0 && leftLimitSwitch.get()) {
        //      output = rotateVelocity;
        //  }
        else if (rotateVelocity < 0.0 && rightLimitSwitch.get()) {
            // output = rotateVelocity; 
            output = 0.0;
        }
        //  else if (rotateVelocity > 0.0 && rightLimitSwitch.get()) {
        //      output = rotateVelocity;
        //  }
        
        m_turretMotor.set(TalonSRXControlMode.PercentOutput, output);
     }
     
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // NetworkTableInstance.getDefault().getEntry("turret/encoder_position")
        //         .setDouble(m_turretMotor.getSelectedSensorPosition());
        // System.out.println("Left  Limit Switch pos " + leftLimitSwitch.get()) ;
        // System.out.println("Right  Limit Switch pos " + rightLimitSwitch.get()) ;
    }

    private void setMotorConfig(WPI_TalonSRX motor) {
        motor.configFactoryDefault();
        motor.configClosedloopRamp(Constants.ShooterConstants.closedVoltageRampingConstant);
        motor.configOpenloopRamp(Constants.ShooterConstants.manualVoltageRampingConstant);
        // motor.config_kF(Constants.ShooterConstants.Turret.PID_id,
        // Constants.ShooterConstants.Turret.kf);
        // motor.config_kP(Constants.ShooterConstants.Turret.PID_id,
        // Constants.ShooterConstants.Turret.kP);
        // motor.config_kI(Constants.ShooterConstants.Turret.PID_id,
        // Constants.ShooterConstants.Turret.kI);
        // motor.config_kD(Constants.ShooterConstants.Turret.PID_id,
        // Constants.ShooterConstants.Turret.kD);
        motor.setNeutralMode(NeutralMode.Brake);
    }

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonSRX(m_turretMotor, 0.5, 6800);
    }

}
