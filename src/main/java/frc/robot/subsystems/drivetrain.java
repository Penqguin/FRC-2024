package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.auto.ReplanningConfig;


public class drivetrain extends TimedRobot {

    // Initializing objects
    private final WPI_TalonSRX leftfront = new WPI_TalonSRX(1);
    private final WPI_TalonSRX leftrear = new WPI_TalonSRX(2);
    private final WPI_TalonSRX rightfront = new WPI_TalonSRX(3);
    private final WPI_TalonSRX rightrear = new WPI_TalonSRX(4);
    private final XboxController controller = new XboxController(0); // Sets up object for the controller

    @Override
    public void robotInit() {
        // Set up motors to follow and be inverted
        leftfront.setInverted(true);
        leftrear.setInverted(true);

        leftrear.follow(leftfront);
        rightrear.follow(rightfront);
    }

    @Override
    public void teleopPeriodic() {
        double Ljoystick = controller.getLeftY();
        double Rjoystick = controller.getRightX();

        if (Ljoystick > 0.1 || Ljoystick < -0.1) {
            Drivecode(Ljoystick, Rjoystick);
        } else if (Rjoystick > 0.1 || Rjoystick < -0.1) {
            Drivecode(Ljoystick, Rjoystick);
        } else {
            Stopdrive();
        }
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
    }

    //pathplanner stuff *not working :__ (

    private void getPose() {
        var gyroAngle = m_gyro.getRotation2d();
        m_pose = m_odometry.update(gyroAngle,
        m_leftEncoder.getDistance(),
        m_rightEncoder.getDistance());
    }

    private void resetPose() {
        new Pose2d(0, 0, new Rotation2d());
    }

    private void getCurrentSpeeds() {
        
    }

    private void Drive() {
        
    }

      AutoBuilder.configureRamsete(
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
    );
}