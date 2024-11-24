package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ControllerRumble extends Command {
    
    private final CommandXboxController controller;
    private final double rumbleValue;
    private final double duration;
    private final Timer timer = new Timer();

    public ControllerRumble(CommandXboxController controller, double rumbleValue, double duration) {
        this.controller = controller;
        this.rumbleValue = rumbleValue;
        this.duration = duration;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        controller.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, rumbleValue);
        controller.getHID().setRumble(GenericHID.RumbleType.kRightRumble, rumbleValue);
    }

    @Override
    public void execute() {
        // Can be used to update rumble dynamically if needed
    }

    @Override
    public void end(boolean interrupted) {
        controller.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, 0);
        controller.getHID().setRumble(GenericHID.RumbleType.kRightRumble, 0);
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration);
    }
}
