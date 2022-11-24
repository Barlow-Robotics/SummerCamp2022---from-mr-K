// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class DriveConstants {
        // All variables are placeholders 
        public static final double kMaxSpeed = 3.0; // meters per second
        public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second
        public static final double DriveSpeed = 0.5;

        public static final double kTrackWidth = 0.381 * 2; // meters
        public static final double kWheelRadius = 0.0508; // meters
        public static final int kEncoderResolution = 4096;
        
        public static final double Gear_Ratio = 1.0 ;
        public static final double Counts_Per_Revolution = kEncoderResolution * Gear_Ratio;
        public static final double InchesToMeters = 0.0254;
        public static final double Wheel_Diameter = 6.0 * InchesToMeters;
        public static final double Meters_Per_Revolution = Wheel_Diameter * Math.PI ;
        public static final double Meters_Per_Count = Meters_Per_Revolution / Counts_Per_Revolution;
        public static final double Counts_Per_Meter = Counts_Per_Revolution / Meters_Per_Revolution;
        public static final double Meters_Per_Second_to_Counts_per_100_mSec = Counts_Per_Meter / 10.0 ;

        public static final double baseDiameter = 33 * InchesToMeters;
        public static final double cirucmferenceWithBD = baseDiameter * Math.PI;
        public static final double wheelBase = 22 * InchesToMeters;
        public static final double circumferenceWithWB = wheelBase * Math.PI;

        public static final double max_velocity_counts = kMaxSpeed * Meters_Per_Second_to_Counts_per_100_mSec ;

        public static final double closedVoltageRampingConstant = 0.0;
        public static final double manualVoltageRampingConstant = 0.0;

        public static final double kF = 0.341;
        public static final double kP = 0.001;
        public static final double kI = 0.0;
        public static final double kD = 0.0;


        public static int ID_leftLeaderMotor = 4;
        public static int ID_leftFollowerMotor = 5;
        public static int ID_rightLeaderMotor = 6;
        public static int ID_rightFollowerMotor = 7;
    }

    public static final class IndexConstants {
        // All variables are placeholders 
        public static final int ID_HopperMotor = 15;
        public static final int ID_FeederMotor = 16;
        public static final int PID_id = 0;

        public static final double closedVoltageRampingConstant = 0;
        public static final double manualVoltageRampingConstant = 0;
        
        public static final double hopperMotorSpeed = 0.25;
        public static final double Hopper_kF = 0.048;
        public static final double Hopper_kP = 0.001;
        public static final double Hopper_kI = 0.0;
        public static final double Hopper_kD = 0.0;

        public static final double Feeder_kF = 0.048;
        public static final double Feeder_kP = 0.001;
        public static final double Feeder_kI = 0.0;
        public static final double Feeder_kD = 0.0;

        public static final double feederMotorSpeed = 0.9;

    }

    public static final class ShooterConstants {
        // All variables are placeholders 

        public static final class Flywheel {
            public static final int ID_Motor = 11;
        
//            public static final double kF = 0.048;
            public static final double kF = 0.059;
            // public static final double kP = 0.001;
            public static final double kP = 0.005;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final int PID_id = 0;

            public static final int UnitsPerMotorRotation = 2048;

            public static final double ChainGearRatio = 24.0 / 42.0; 
            public static final double UnitsPerFlywheelRotation = UnitsPerMotorRotation / ChainGearRatio ;

            public static final double RPM = ChainGearRatio / 60.0 * UnitsPerMotorRotation / 10.0 ;
            public static final double FlywheelVelocity = 5000 * RPM ;
//            public static final double FlywheelVelocity = 10000 * RPM ;
        }

        public static final double closedVoltageRampingConstant = 0.1;
        public static final double manualVoltageRampingConstant = 0.2;
    
        public static class Turret {
            public static final int ID_Motor = 10;

            public static final double maxTurretOutput = 0.15 ; 
            public static final double kp = 0.4 / 160.0 ;  // maximum error. 
            public static final double ki = 0.005 ; 
            public static final double kd = kp * 0.125 ; 
            public static final double AlignmentTolerence = 10.0;

            public static final int ID_LeftLimitSwitch = 0;
            public static final int ID_RightLimitSwitch = 1;
        }

        public static class Hood {
            public static final int ID_leftServo = 0;
            public static final int ID_rightServo = 1;
            public static final double maxHoodOutput = 0;
            public static final double servoPos = 0;
        }
    }

    public static final class UnderGlowConstants {
    }

    public static final class VisionConstants {
    }

    public static final class AutoConstants {
    }

    public final class Logitech_Dual_Action {

        // Constants for Axes
        public static final int Left_Stick_X = 0;
        public static final int Left_Stick_Y = 1;
        public static final int Right_Stick_X = 2;
        public static final int Right_Stick_Y = 3;

        // Constants for buttons
        public static final int Left_Trigger = 7;
        public static final int Right_Trigger = 8;
        public static final int Button_A = 2;
        public static final int Button_B = 3;
        public static final int Button_X = 1;
        public static final int Button_Y = 4;
        public static final int Left_Bumper = 5;
        public static final int Right_Bumper = 6;
        public static final int Back_Button = 9;
        public static final int Start_Button = 10;
        public static final int Left_Stick = 11;
        public static final int Right_Stick = 12;

        public static final double Forward_Axis_Attenuation = -0.5 ;
        public static final double Lateral_Axis_Attenuation = 0.5 ;
        public static final double Yaw_Axis_Attenuation = 0.5 ;
    }
    
    public final class Logitech_F310_Controller {
        // Constants for Axes
        public static final int Left_Stick_X = 0;
        public static final int Left_Stick_Y = 1;
        public static final int Left_Trigger = 2;
        public static final int Right_Trigger = 3;
        public static final int Right_Stick_X = 4;
        public static final int Right_Stick_Y = 5;

        // Constants for buttons
        public static final int Button_A = 1;
        public static final int Button_B = 2;
        public static final int Button_X = 3;
        public static final int Button_Y = 4;
        public static final int Left_Bumper = 5;
        public static final int Right_Bumper = 6;
        public static final int Back_Button = 7;
        public static final int Start_Button = 8;
        public static final int Left_Stick = 9;
        public static final int Right_Stick = 10;

        public static final double Forward_Axis_Attenuation = -0.5 ;
        public static final double Lateral_Axis_Attenuation = 0.5 ;
        public static final double Yaw_Axis_Attenuation = 0.5 ;
    }
}
