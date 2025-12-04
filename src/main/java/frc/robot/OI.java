package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OI {
    private static OI instance;
    private PS4Controller controller;

    public OI() {
        controller = new PS4Controller(0);
        configureController();
    }

    public static OI getInstance() {
        if (instance == null)
            instance = new OI();
        return instance;
    }

    public void configureController() {
        controller = new PS4Controller(0);

        Trigger squareButton = new JoystickButton(controller, PS4Controller.Button.kSquare.value);
        squareButton.toggleOnTrue(new LimelightOnCommand());
    }
}
