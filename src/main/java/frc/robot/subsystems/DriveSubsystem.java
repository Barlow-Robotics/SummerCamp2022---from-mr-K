// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.sim.PhysicsSim;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Represents a differential drive style drivetrain. */
public class DriveSubsystem extends SubsystemBase {
    WPI_TalonSRX m_leftLeader;
    WPI_VictorSPX m_leftFollower;
    WPI_TalonSRX m_rightLeader;
    WPI_VictorSPX m_rightFollower;

    DifferentialDrive diffDrive;

    private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();
    private final ADXRS450_GyroSim gyroSim = new ADXRS450_GyroSim(m_gyro);

    private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(
            Constants.DriveConstants.kTrackWidth);
    private final DifferentialDriveOdometry m_odometry;

    boolean simulationInitialized = false;

    // Gains are for example purposes only - must be determined for your own robot!
    // private final SimpleMotorFeedforward m_feedforward = new
    // SimpleMotorFeedforward(1, 3);

    /**
     * Constructs a differential drive object. Sets the encoder distance per pulse
     * and resets the
     * gyro.
     */
    public DriveSubsystem() {

        m_leftLeader = new WPI_TalonSRX(Constants.DriveConstants.ID_leftLeaderMotor);
        m_leftFollower = new WPI_VictorSPX(Constants.DriveConstants.ID_leftFollowerMotor);
        m_rightLeader = new WPI_TalonSRX(Constants.DriveConstants.ID_rightLeaderMotor);
        m_rightFollower = new WPI_VictorSPX(Constants.DriveConstants.ID_rightFollowerMotor);

        // Config Motors
        setMotorConfig( m_leftLeader) ;
        setMotorConfig( m_rightLeader) ;
        // m_leftLeader.configFactoryDefault();
        // m_rightLeader.configFactoryDefault();
        m_leftFollower.configFactoryDefault();
        m_rightFollower.configFactoryDefault();
        m_leftFollower.follow(m_leftLeader);
        m_rightFollower.follow(m_rightLeader);
        m_rightLeader.setInverted(InvertType.InvertMotorOutput);
        m_rightFollower.setInverted(InvertType.FollowMaster);
        m_leftFollower.setInverted(InvertType.FollowMaster);

        m_leftLeader.setSensorPhase(true);
//        m_rightLeader.setSensorPhase(true);
        m_rightLeader.setSensorPhase(false);

         diffDrive = new DifferentialDrive(m_leftLeader, m_rightLeader);

        m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

        setDefaultNeutralMode();
        m_gyro.reset();
        CreateNetworkTableEntries();
    }




    private void setMotorConfig(WPI_TalonSRX motor) { // changed to TalonFX for intake
        motor.configFactoryDefault();
        motor.configClosedloopRamp(Constants.DriveConstants.closedVoltageRampingConstant);
        motor.configOpenloopRamp(Constants.DriveConstants.manualVoltageRampingConstant);
        motor.config_kF(0, Constants.DriveConstants.kF);
        motor.config_kP(0, Constants.DriveConstants.kP);
        motor.config_kI(0, Constants.DriveConstants.kI);
        motor.config_kD(0, Constants.DriveConstants.kD);
        motor.setNeutralMode(NeutralMode.Brake);

        // wpk not sure we need the line below. Add in if it doesn't seem to be working.

        // 		/* Config sensor used for Primary PID [Velocity] */
        // motor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
    }

    public void periodic() {
        m_odometry.update(
                m_gyro.getRotation2d(),
                getLeftDistance(),
                getRightDistance());

        Pose2d pose = m_odometry.getPoseMeters() ;
        NetworkTableInstance.getDefault().getEntry("drive/pose/x").setDouble(pose.getX());
        NetworkTableInstance.getDefault().getEntry("drive/pose/y").setDouble(pose.getY());
        NetworkTableInstance.getDefault().getEntry("drive/pose/rotation").setDouble(pose.getRotation().getDegrees());
        
    }

    public void setDefaultNeutralMode() {
        m_leftLeader.setNeutralMode(NeutralMode.Brake);
        m_rightLeader.setNeutralMode(NeutralMode.Brake);
        // m_leftLeader.setNeutralMode(NeutralMode.Coast);
        // m_rightLeader.setNeutralMode(NeutralMode.Coast);
    }

    /**
     * Sets the desired wheel speeds.
     *
     * @param speeds The desired wheel speeds.
     */
    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        m_leftLeader.set(Constants.DriveConstants.DriveSpeed);
        m_rightLeader.set(Constants.DriveConstants.DriveSpeed);
    }

    private double getLeftSpeed() {
        double s = m_leftLeader.getSelectedSensorVelocity() * 10.0 * Constants.DriveConstants.Meters_Per_Count;
        return (s);
    }

    private double getRightSpeed() {
        double s = -m_rightLeader.getSelectedSensorVelocity() * 10.0 * Constants.DriveConstants.Meters_Per_Count;
        return (s);
    }

    public double getLeftDistance() {
        double d = (m_leftLeader.getSelectedSensorPosition() / Constants.DriveConstants.Counts_Per_Revolution)
                * Constants.DriveConstants.Meters_Per_Revolution;
        return (d);
    }

    public double getRightDistance() {
        double d = (-m_rightLeader.getSelectedSensorPosition() / Constants.DriveConstants.Counts_Per_Revolution)
                * Constants.DriveConstants.Meters_Per_Revolution;
        return (d);
    }

    /**
     * Drives the robot with the given linear velocity and angular velocity.
     *
     * @param xSpeed       Linear velocity in m/s.
     * @param rot          Angular velocity in rad/s.
     * @param squareInputs Decreases input sensitivity at low speeds.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double rot, boolean squareInputs) {
        NetworkTableInstance.getDefault().getEntry("drive/xSpeed").setDouble(m_leftLeader.getSelectedSensorVelocity());
        NetworkTableInstance.getDefault().getEntry("drive/rot").setDouble(m_leftLeader.getSelectedSensorPosition());
        NetworkTableInstance.getDefault().getEntry("drive/distance").setDouble(getLeftDistance());

        // NetworkTableInstance.getDefault().getEntry("drive/arcadeDrive").setDouble(100.0);
        // diffDrive.arcadeDrive(xSpeed, rot, squareInputs);

        DifferentialDrive.WheelSpeeds relativeSpeeds = DifferentialDrive.arcadeDriveIK(xSpeed, rot, squareInputs) ;
        DifferentialDrive.WheelSpeeds absoluteSpeeds 
           = new DifferentialDrive.WheelSpeeds( 
                 relativeSpeeds.left * Constants.DriveConstants.kMaxSpeed, //MaxAttainableVelocity
                 relativeSpeeds.right * Constants.DriveConstants.kMaxSpeed);
        setWheelSpeeds( absoluteSpeeds ) ;
    }


    public void resetEncoders() {
        m_leftLeader.setSelectedSensorPosition(0);
        m_rightLeader.setSelectedSensorPosition(0);
    }

//     public void setMaxOutput(double maxOutput) {
//         diffDrive.setMaxOutput(maxOutput);
//     }

    public double getGyroHeading() {
        return Math.IEEEremainder(m_gyro.getAngle(), 360.0);
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(getGyroHeading());
    }

    public void resetHeading() {
        m_gyro.reset();
    }


    public void setWheelSpeeds( DifferentialDrive.WheelSpeeds speeds) {
        // m_leftLeader.set( TalonSRXControlMode.Velocity, (speeds.left*Constants.DriveConstants.Counts_Per_Meter*(1/100))) ;
        // m_rightLeader.set( TalonSRXControlMode.Velocity, (speeds.right*Constants.DriveConstants.Counts_Per_Meter*(1/100)));

        m_leftLeader.set( TalonSRXControlMode.Velocity, speeds.left * Constants.DriveConstants.Meters_Per_Second_to_Counts_per_100_mSec) ;
        m_rightLeader.set( TalonSRXControlMode.Velocity, speeds.right * Constants.DriveConstants.Meters_Per_Second_to_Counts_per_100_mSec ) ;
    }


    public void setWheelSpeeds( double left, double right) {
        setWheelSpeeds(new DifferentialDrive.WheelSpeeds(left, right));
    }


    public DifferentialDrive.WheelSpeeds getWheelSpeeds() {
        DifferentialDrive.WheelSpeeds result = new DifferentialDrive.WheelSpeeds() ;
        result.left = getLeftSpeed() ;
        result.right = getRightSpeed() ;

        return result ;
    }



    // private double getSpeed(WPI_TalonFX motor) {
    //     // multiplied by 10 because velocity reported as counts per 1/10th second
    //     double s = motor.getSelectedSensorVelocity() * 10.0 * Constants.DriveConstants.Meters_Per_Count;
    //     return (s);
    // }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(pose, m_gyro.getRotation2d());
    }




    private void CreateNetworkTableEntries() {
        NetworkTableInstance.getDefault().getEntry("drive/left_motor_distance").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("drive/right_motor_distance").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("drive/rotation").setDouble(0.0);

        NetworkTableInstance.getDefault().getEntry("drive/leftSpeed").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("drive/rightSpeed").setDouble(0.0);

        NetworkTableInstance.getDefault().getEntry("drive/xSpeed").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("drive/rot").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("drive/arcadeDrive").setDouble(0.0);

        NetworkTableInstance.getDefault().getEntry("drive/leftVolts").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("drive/rightVolts").setDouble(0.0);

    }

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonSRX(m_leftLeader, 0.75, 6800, false);
        PhysicsSim.getInstance().addVictorSPX(m_leftFollower);
        PhysicsSim.getInstance().addTalonSRX(m_rightLeader, 0.75, 6800, false);
        PhysicsSim.getInstance().addVictorSPX(m_rightFollower);
    }

    @Override
    public void simulationPeriodic() {
        if (!simulationInitialized) {
            simulationInit();
            simulationInitialized = true;
        }
        PhysicsSim.getInstance().run();

        double headingNoise = 0.0; // (Math.random() - 0.5) * 4.0 ;
        // gyroSim.setAngle(this.m_odometry.getPoseMeters().getRotation().getDegrees() +
        // headingNoise);
        gyroSim.setAngle(5.0);
        gyroSim.setRate(1.0);
        NetworkTableInstance.getDefault().getEntry("drive/gyro/getAngle").setDouble(m_gyro.getAngle());
    }



}
