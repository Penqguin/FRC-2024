package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

// import com.pathplanner.lib.auto.ReplanningConfig;


public class drivetrain extends TimedRobot {

    private static drivetrain drivetrain = null; //put this if you want the robot to periodically call this
    // Initializing objects

    //Motors and Controller
    private WPI_TalonSRX leftfront;
    private WPI_TalonSRX leftrear;
    private WPI_TalonSRX rightfront;
    private WPI_TalonSRX rightrear;

    //Gyro stuff
    private AHRS navx;
    private Encoder rightEncoder;
    private Encoder leftEncoder;
    // private final Rotation2d rotation2d = new Rotation2d();
    private DifferentialDriveOdometry odometry;
    public AutoBuilder autoBuilder;
    
    private drivetrain(){
        leftfront = new WPI_TalonSRX(1);
        leftrear = new WPI_TalonSRX(2);
        rightfront = new WPI_TalonSRX(3);
        rightrear = new WPI_TalonSRX(4);
        navx = new AHRS(SPI.Port.kMXP);
        rightEncoder = new Encoder(0,1);
        leftEncoder = new Encoder(0,2);
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(navx.getYaw()), leftEncoder.getDistance(), rightEncoder.getDistance());
        autoBuilder = new AutoBuilder();

        leftfront.setInverted(true);
        leftrear.setInverted(true);

        leftrear.follow(leftfront);
        rightrear.follow(rightfront);
        
        resetPose();

    }

    // Methods
    private void Stopdrive() {
        leftfront.set(0);
        rightfront.set(0);
    }

    public void Drivecode(double Leftjoy, double Rightjoy) {
        if (Leftjoy > 0.1 || Leftjoy < -0.1) {
            leftfront.set(Leftjoy);
            rightfront.set(Leftjoy);
        } else if (Rightjoy > 0.1) {
            leftfront.set(Rightjoy);
            rightfront.set(-Rightjoy / 1.5);
        } else if (Rightjoy < -0.1) {
            leftfront.set(-Rightjoy / 1.5);
            rightfront.set(Rightjoy);
        } else {
            Stopdrive();
        }
    

        //pathplanner stuff *not working yet :__ (

        autoBuilder.configureRamsete(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getCurrentSpeeds, // Current ChassisSpeeds supplier
            this::drive, // Method that will drive the robot given ChassisSpeeds
            new ReplanningConfig(), // Default path replanning config. See the API for the options here
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );}

    
    //auto methods:

    //getPosition (pose)
    public void getPose(){
        var gyroAngle = navx.getRotation2d();
        odometry.update(gyroAngle, leftEncoder.getDistance(), rightEncoder.getDistance() ); 
    }

    //resetPosition (reset pose)
    public void resetPose() {
        leftEncoder.reset();
        rightEncoder.reset();
        navx.reset();
        var gyroAngle = navx.getRotation2d();
        odometry.update(gyroAngle, leftEncoder.getDistance(), rightEncoder.getDistance() ); 
    }

    //get Chasis speed (speed methods not working because I need to integrate more variables and functions)

    public ChassisSpeeds getCurrentSpeeds() {
        double leftSpeed = leftEncoder.getRate();
        double rightSpeed = rightEncoder.getRate();
        double angularVelocity = Math.toRadians(navx.getRate()); // Get angular velocity from NavX in rad/s
        return new ChassisSpeeds(leftSpeed, rightSpeed, angularVelocity);
    }

    public void drive(double xSpeed, double rot) {
        var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
        setSpeeds(wheelSpeeds);
      }




    //sends to operator interface
    public static drivetrain getInstance(){
        if (drivetrain == null){
            drivetrain = new drivetrain();
        }
        return drivetrain;
    }
}

    //Get Chasis speed
   

    //Drive with current chasis speed

    /*
     public void drive(chasis speed){
        Motor.set(chasis speed)
        Motor.set(chasis speed)
     }



     */



