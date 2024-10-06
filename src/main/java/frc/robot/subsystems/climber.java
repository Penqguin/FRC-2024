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

    private int[] climbState = {1,2}; // Initializes the climb state as down
    private int climbIndex = 1;
    private boolean pastButtonn = false;

    public void Climber(boolean button){
        if(button != pastButtonn){
            pastButtonn = button; // acknowledge that the button has been pressed
            if(climbIndex == 1){
                climbIndex = 0;
            }
            else if(climbIndex == 0){
                climbIndex = 1;
            }
        }

       if(button && climbState[climbIndex] == 1){
            ClimbMotor.set(1);
            // climbIndex =2;
        }
        else if(button && climbState[climbIndex] == 2) {
            ClimbMotor.set(-1);
            // climbIndex = 1;
        }
        else{
            stopMotor();//sigmasigma sigma sigma sigma sigma
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