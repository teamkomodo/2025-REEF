package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.BlinkinPattern;

import static frc.robot.Constants.*;

public class LEDSubsystem extends SubsystemBase {
    public static final double IDLE_PATTERN = 0.43; //Sparkle, Color 1 on Color 2

    private Spark frameLights = new Spark(LED_CHANNEL);

    private double framePattern = 0;

    public void setFramePattern(double pattern){
        frameLights.set(pattern);
    }

    public Command setTempFramePatternCommand(double pattern) {
        return Commands.runEnd(() -> setFramePattern(pattern), () -> setFramePattern(framePattern));
    }

    public Command setFramePatternCommand(double pattern) {
        return Commands.runOnce(() -> { setFramePattern(pattern); framePattern = pattern;});
    }
}
