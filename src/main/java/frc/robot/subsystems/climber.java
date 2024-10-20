package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.button.JoystickButton; // unused rn
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;


public class climber extends SubsystemBase {
    private static climber climber = null; //Use if periodically updated to the robot (operator interface)

    private WPI_VictorSPX ClimbMotor; // Sets up object representing the real arm motor 
    
    public climber() {
        ClimbMotor = new WPI_VictorSPX(7); // Initializes the motor controller
    }

    public void Climber(boolean button1, boolean button2){
       if(button1){
            ClimbMotor.set(1);
            //climber up
        }
        else if(button2) {
            ClimbMotor.set(-1);
            //climber down
        }
        else{
            stopMotor();
        }
    }

    // Our Methods
    private void stopMotor() {
        ClimbMotor.set(0);
    }

    //send to operator interface
    public static climber getInstance(){
        if (climber == null){
            climber = new climber();
        }
        return climber;
    }
}