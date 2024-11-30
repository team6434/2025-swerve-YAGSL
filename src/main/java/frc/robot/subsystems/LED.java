package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase{
    
    public Spark blinkin;

    public LED() {
        blinkin = new Spark(0);
    }

    public void setOrange() {
        blinkin.set(0.65);
    }

    public void setBlue() {
        blinkin.set(0.87);
    }

    public void setRed() {
        blinkin.set(0.61);
    }

    public void setGreen() {
        blinkin.set(0.77);
    }

    public void setPurple() {
        blinkin.set(0.91);
    }

    public void setPink() {
        blinkin.set(0.57);
    }

    public void setRainbow() {
        blinkin.set(-0.99);
    }
}
