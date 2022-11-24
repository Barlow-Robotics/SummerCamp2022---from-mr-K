// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import java.util.*;

public class DriveDistance extends CommandBase {

    DriveSubsystem drive;
    double targetDistance;
    double speed ;

    double startingLeftDistance ;
    double startingRightDistance ;


    /** Creates a new DriveDistance. */
    public DriveDistance(DriveSubsystem d, double dist, double s) {
        // Use addRequirements() here to declare subsystem dependencies.
        drive = d;
        targetDistance = dist;
        speed = s;
        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        drive.resetOdometry(new Pose2d());
        startingLeftDistance = drive.getLeftDistance();
        startingRightDistance = drive.getRightDistance();

       // NetworkTableInstance.getDefault().getEntry("initialEncoderCounts").setDouble(get.Pose2d());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        drive.setWheelSpeeds(new DifferentialDrive.WheelSpeeds(speed, speed));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drive.setWheelSpeeds(new DifferentialDrive.WheelSpeeds(0.0, 0.0));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        double distanceTraveled = (( drive.getLeftDistance() - startingLeftDistance) + ( drive.getRightDistance() - startingRightDistance))/2.0 ;
        if (Math.abs(distanceTraveled) >= Math.abs(targetDistance)) {
            return true ;
        }
        return false;
    }
}
