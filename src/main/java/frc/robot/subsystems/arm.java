package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton; 
//These are underlined red because we need to get the vendor libraries, !somebody download them please!
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class arm extends SubsystemBase{
    //check if the driver is pressing the toggle button
    //if they aren't don't do anything
    //if the toggle button is pressed, check if the arm state is up or down, depending on what it is do the opposite

    private final WPI_VictorSPX ArmMotor = new WPI_VictorSPX(5); //Sets up object representing the real arm motor
    private final XboxController controller = new XboxController(0); //Sets up object for the controller

    boolean armstate = false; //initalizes the arm state as down
    boolean aPress = controller.getAButtonPressed();
    
    if(aPress)
    {
        if(armstate == false){
            ArmMotor.set(1.0);
            armstate = true;
        }
        else{
            ArmMotor.set(0);
            armstate = false;
        }
    }
    else
    {
        Stopmotor(); //stops the motor
    }

    //Our Methods
    private void Stopmotor(){
        ArmMotor.set(0);
    }
    

}