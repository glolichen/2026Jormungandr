package frc.robot;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class Limelight implements Subsystem {
    private static final String name = "limelight-climber";
    private static Limelight llClimber;

    private Limelight() { }

    public static Limelight getInstance() {
        if (llClimber == null)
            llClimber = new Limelight();
        return llClimber;
    }

    public void setLight(boolean on) {
        if (on)
            LimelightHelpers.setLEDMode_ForceOn(name);
        else
            LimelightHelpers.setLEDMode_ForceOff(name);
    }

    @Override
    public void periodic() {

    }
}
