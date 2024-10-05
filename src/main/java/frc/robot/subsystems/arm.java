package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton; // unused rn
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class arm extends SubsystemBase {
    // Check if the driver is pressing the toggle button
    // If they aren't, don't do anything
    // If the toggle button is pressed, check if the arm state is up or down, depending on what it is, do the opposite

    private final WPI_VictorSPX ArmMotor = new WPI_VictorSPX(5); // Sets up object representing the real arm motor
    private final XboxController controller = new XboxController(0); // Sets up object for the controller

    private boolean armState = false; // Initializes the arm state as down

    public arm() {
        // Constructor
    }

    @Override
    public void periodic() {
        boolean aPress = controller.getAButtonPressed();

        if (aPress) {
            if (armState == false) {
                ArmMotor.set(1.0);
                armState = true;
            } else {
                ArmMotor.set(0);
                armState = false;
            }
        } else {
            stopMotor(); // Stops the motor
        }
    }

    // Our Methods
    private void stopMotor() {
        ArmMotor.set(0);
    }
}