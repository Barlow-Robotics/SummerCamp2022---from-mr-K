// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hood extends SubsystemBase {

    Servo leftServo;
    Servo rightServo;
    double currentPosition = 0.5;

    // public ArrayList<Double> hoodPositions;

    /** Creates a new Hood. */
    public Hood() {
        leftServo = new Servo(Constants.ShooterConstants.Hood.ID_leftServo);
        rightServo = new Servo(Constants.ShooterConstants.Hood.ID_rightServo);
        leftServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
        leftServo.enableDeadbandElimination(true);
        rightServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
        rightServo.enableDeadbandElimination(true);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        leftServo.set(currentPosition);
        rightServo.set(currentPosition);
    }

    public void setServoPosition(double pos) {
        currentPosition = pos;
    }

    public void movePosition(double deltaPos) {
        currentPosition = currentPosition + deltaPos;
        if (currentPosition >= 1.0) {
            currentPosition = 1.0;
        }

        if (currentPosition <= 0.0) {
            currentPosition = 0.0;
        }
    }
}
