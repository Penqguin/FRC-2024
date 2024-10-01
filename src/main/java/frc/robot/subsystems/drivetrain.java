package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton; 
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class drivetrain extends SubsystemBase{

    //Get the users left and right joystick inputs
    //check which joystick is in use (if it's both don't move at all)
    //if left is being used set the motors to the value
    //if right is being used set the motors to opposite to turn

    //initializing objects

    private final WPI_TalonSRX leftfront = new WPI_TalonSRX(1);
    private final WPI_TalonSRX leftrear = new WPI_TalonSRX(2);
    private final WPI_TalonSRX rightfront = new WPI_TalonSRX(3);
    private final WPI_TalonSRX rightrear = new WPI_TalonSRX(4);
    private final XboxController controller = new XboxController(0); //Sets up object for the controller

    //all code after this assumes robot init sets up all the motors to follow and be inverted
    //if this code doesn't work you need to implement something like this
    /*
        private void setMotor(){
        leftfront.setInverted(true);
        leftrear.setInverted(true);

        leftrear.follow(leftfront);
        rightrear.follow(rightfront);
        }
    */ 
    
    double Ljoystick = controller.getLeftY();
    double Rjoystick = controller.getRightX();

        if(Ljoystick > 0.1 || Ljoystick < -0.1){
            Drivecode(Ljoystick, Rjoystick);
        }
        else if(Rjoystick > 0.1 || Rjoystick < -0.1){
            Drivecode(Ljoystick, Rjoystick);
        }
        else{
            Stopdrive();
        }

    //Methods
    private void Stopdrive(){
        leftfront.set(0);
        rightfront.set(0);
    }

    public void Drivecode(double Leftjoy, double Rightjoy){
        if (Leftjoy > 0.1 || Leftjoy < -0.1) {
          leftfront.set(Leftjoy);
          rightfront.set(Leftjoy);
        } 
        else if (Rightjoy > 0.1) {
          leftfront.set(Rightjoy);
          rightfront.set(-Rightjoy / 1.5);
        } 
        else if (Rightjoy < -0.1) {
          leftfront.set(-Rightjoy / 1.5);
          rightfront.set(Rightjoy);
        } 
        else {
          Stopdrive();
        }
      }  
}