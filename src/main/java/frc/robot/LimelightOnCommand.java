package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

public class LimelightOnCommand extends Command {
    private Limelight ll;

    public LimelightOnCommand() {
        ll = Limelight.getInstance();
    }

    @Override
    public void initialize() {
        ll.setLight(true);
    }
    
    @Override
    public void end(boolean interrupted) {
        ll.setLight(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
