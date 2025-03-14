package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

    public Command flashGreenCommand() {
        return new SequentialCommandGroup(Commands.runOnce(() -> setFramePattern(BlinkinPattern.SOLID_COLORS_GREEN)),
        Commands.waitSeconds(0.1),
        Commands.runOnce(() -> setFramePattern(BlinkinPattern.SOLID_COLORS_GREEN)),
        setTempFramePatternCommand(IDLE_PATTERN));
    }

    public Command flashOrangeCommand() {
        return new SequentialCommandGroup(Commands.runOnce(() -> setFramePattern(BlinkinPattern.SOLID_COLORS_ORANGE)),
        Commands.waitSeconds(0.1),
        Commands.runOnce(() -> setFramePattern(BlinkinPattern.SOLID_COLORS_ORANGE)),
        setTempFramePatternCommand(IDLE_PATTERN));
    }

    public Command flashPinkCommand() {
        return new SequentialCommandGroup(Commands.runOnce(() -> setFramePattern(BlinkinPattern.SOLID_COLORS_HOT_PINK)),
        Commands.waitSeconds(0.1),
        Commands.runOnce(() -> setFramePattern(BlinkinPattern.SOLID_COLORS_HOT_PINK)),
        setTempFramePatternCommand(IDLE_PATTERN));
    }
    


    


   
}
