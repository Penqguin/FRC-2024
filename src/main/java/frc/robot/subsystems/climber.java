package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.button.JoystickButton; // unused rn
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class climber extends SubsystemBase {
    // Check if the driver is pressing the toggle button
    // If they aren't, don't do anything
    // If the toggle button is pressed, check if the arm state is up or down, depending on what it is, do the opposite

    private final WPI_VictorSPX ClimbMotor = new WPI_VictorSPX(6); // Sets up object representing the real arm motor
    private final XboxController controller = new XboxController(0); // Sets up object for the controller

    private int[] climbState = {1,2}; // Initializes the climb state as down
    private int climbIndex = 1;

    public climber() {
        // Constructor
    }

    

    @Override
    public void periodic() {
        boolean xPress = controller.getXButtonPressed();

        if (xPress) {
            if (climbState[climbIndex] == 1 && xPress == true) {
                ClimbMotor.set(1.0);
                climbIndex = 2;
            } 
            else if(climbState[climbIndex] == 2 && xPress == true) {
                ClimbMotor.set(-1.0);
                climbIndex = 1;
            }
           else {
                stopMotor();
            }
        }
    }

    // Our Methods
    private void stopMotor() {
        ClimbMotor.set(0);
    }
}