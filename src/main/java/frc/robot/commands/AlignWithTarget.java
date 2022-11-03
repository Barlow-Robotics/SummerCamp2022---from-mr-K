// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Vision;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Turret;

import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AlignWithTarget extends CommandBase {

    private PIDController pid = new PIDController(Constants.ShooterConstants.Turret.kp, 0, 0);
    private Vision m_vision;
    private Turret m_turret;
    private Hood m_hood;

    int missedFrames = 0;
    private boolean alignmentComplete = false;

    private double hoodPositionLUT[] = new double[50];

    class DistanceLUTPair {
        public double height;
        public int distance;

        public DistanceLUTPair(double h, int d) {
            height = h;
            distance = d;
        }
    }

    private ArrayList<DistanceLUTPair> distanceLUT = new ArrayList<DistanceLUTPair>();

    /** Creates a new AlignWithTarget. */
    public AlignWithTarget(Turret t, Vision v, Hood h) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_vision = v;
        m_turret = t;
        m_hood = h;

        addRequirements(m_turret, m_hood);

        distanceLUT.add(new DistanceLUTPair(320.0, 8));
        distanceLUT.add(new DistanceLUTPair(105.0, 8));
        distanceLUT.add(new DistanceLUTPair(100.0, 9));
        distanceLUT.add(new DistanceLUTPair(92.0, 10));
        distanceLUT.add(new DistanceLUTPair(79.5, 11));        
        distanceLUT.add(new DistanceLUTPair(76.0, 12));        
        distanceLUT.add(new DistanceLUTPair(72.0, 13));        
        distanceLUT.add(new DistanceLUTPair(65.0, 14));        
        distanceLUT.add(new DistanceLUTPair(60.5, 15));        
        distanceLUT.add(new DistanceLUTPair(56.6, 16));        
        distanceLUT.add(new DistanceLUTPair(55.5, 17));        
        distanceLUT.add(new DistanceLUTPair(52.3, 18));        
        distanceLUT.add(new DistanceLUTPair(51.5, 19));
        distanceLUT.add(new DistanceLUTPair(49.0, 20));
        distanceLUT.add(new DistanceLUTPair(0, 20));

        

        for (int i = 0; i < 10; i++) {
            hoodPositionLUT[i] = 0.0;
        }

        hoodPositionLUT[8] = 1;
        hoodPositionLUT[9] = 0.825;
        hoodPositionLUT[10] = 0.65;
        hoodPositionLUT[11] = 0.575;
        hoodPositionLUT[12] = 0.5;
        hoodPositionLUT[13] = 0.375;
        hoodPositionLUT[14] = 0.25;
        hoodPositionLUT[15] = 0.2;
        hoodPositionLUT[16] = 0.15;
        hoodPositionLUT[17] = 0.1;
        hoodPositionLUT[18] = 0.05;
        hoodPositionLUT[19] = 0.025;
        hoodPositionLUT[20] = 0.0;

        for (int i = 21; i < hoodPositionLUT.length; i++) {
            hoodPositionLUT[i] = 0.0;
        }

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        pid.reset();
        missedFrames = 0;
        alignmentComplete = false;
    }

    int getDistanceToTarget(double boxHeight) {
            
        int returnVal = 0;

        for ( int i = 0; i < distanceLUT.size(); i++) {
            if ( boxHeight > distanceLUT.get(i).height ) {
                returnVal = distanceLUT.get(i).distance ;
                break ;
            }
        }

        return returnVal;
    }

    double getHoodPosition(int distanceToTarget) {
        return hoodPositionLUT[distanceToTarget] ;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // if (m_vision.visionTargetIsVisible()) {
        // System.out.println("Target is visible");
        // } else {
        // System.out.println("Target is not visible");
        // }

        double rotateVelocity = 0.0;
        double azimuthError = 0.0;

        if (m_vision.visionTargetIsVisible()) {
            azimuthError = -m_vision.visionTargetDistanceFromCenter();
            if (Math.abs(azimuthError) < Constants.ShooterConstants.Turret.AlignmentTolerence) {
                alignmentComplete = true;
            } else {
                double adjustment = pid.calculate(azimuthError);
                adjustment = Math.signum(adjustment)
                        * Math.min(Math.abs(adjustment), Constants.ShooterConstants.Turret.maxTurretOutput);
                rotateVelocity = adjustment;
            }
            // m_turret.rotate(0.0);
            // System.out.println("Alignment adjustment is " + rotateVelocity);
            m_turret.rotateTurret(rotateVelocity);

            m_hood.setServoPosition(getHoodPosition(getDistanceToTarget(m_vision.bbHeight())));
        

        } else {
            // System.out.println("Target not visible");
            missedFrames++;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_turret.rotateTurret(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
        // if (missedFrames > 10) {
        // return true;
        // } else {
        // return false;
        // }
    }
}
