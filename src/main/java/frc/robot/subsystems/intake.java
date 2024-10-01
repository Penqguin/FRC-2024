package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton; 
//These are underlined red because we need to get the vendor libraries, !somebody download them please!
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class intake extends SubsystemBase{
    //check if the driver is pressing the toggle button
    //if they aren't don't do anything
    //if the toggle button is pressed, check if the arm state is up or down, depending on what it is do the opposite

    private final WPI_VictorSPX RightMotor = new WPI_VictorSPX(5); //Sets up object representing the real arm motor
    private final WPI_VictorSPX LeftMotor = new WPI_VictorSPX(6); //Sets up object representing the real arm motor
    private final XboxController controller = new XboxController(0); //Sets up object for the controller

    
    double rightTrigger = controller.getRightTriggerAxis();
    double leftTrigger = controller.getLeftTriggerAxis();
    
    if(rightTrigger > 0.1 && rightTrigger > leftTrigger)
    {
        RightMotor.set(rightTrigger);     
    }
    else if (leftTrigger > 0.1 && leftTrigger > rightTrigger){
        LeftMotor.set(leftTrigger);  
    }
    else
    {
        Stopinktake(); //stops the motor
    }

    //Our Methods
    private void Stopinktake(){
        RightMotor.set(0);
        LeftMotor.set(0);
    }

}