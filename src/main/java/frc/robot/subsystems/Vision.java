// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */

  boolean targetIsVisible = false;

  public void LookUpTable(){}
  public ArrayList<Double> lookUpTArrayList;
  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean visionTargetIsVisible() {
    // The data for this will come from the Jetson Nano via network tables.
    return NetworkTableInstance.getDefault().getEntry("vision/target_detected").getBoolean(false);
  }

  public double visionTargetDistanceFromCenter() {
    // returns the number of pixels from the center of the screen to the center of
    // the vision target.
    // The data for this will come from the Jetson Nano via network tables.
    return NetworkTableInstance.getDefault().getEntry("vision/target_distance_from_center").getDouble(0.0);
  }

  public double bbHeight() {
    return NetworkTableInstance.getDefault().getEntry("vision/target_bb_height").getDouble(0.0);
  }

  public double bbWidth() {
    return NetworkTableInstance.getDefault().getEntry("vision/target_bb_width").getDouble(0.0);
  }
}
