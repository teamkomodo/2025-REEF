package frc.robot.util;


public class Util {



    public static double translationCurve(double input) {
        return Math.pow(Math.abs(input), 1.5) * Math.signum(input);
        // double magnitude = Math.abs(input);
        // if(magnitude < 0.5) {
        //     return Math.signum(input) * (Math.pow(magnitude, 3) + .25 * input);
        // }
        
        // return Math.signum(input) * (-1 * Math.pow(magnitude, 3) + 3 * Math.pow(magnitude, 2) - 1.25 * magnitude + .25);
    }

    public static double steerCurve(double input) {
        return Math.pow(input, 3);
    }

    // public static void setPidController(SparkClosedLoopController pidController, PIDGains pid) {
    //     SparkMaxConfig Sparkconfig = new SparkMaxConfig();
    //     switch (pid.numArgs) {
    //         case 8:
    //             pidController.setOutputRange(pid.minOutput, pid.maxOutput);
    //             pidController.setFF(pid.FF);
    //             pidController.setIZone(pid.iZone);
    //         case 4:
    //             pidController.setIMaxAccum(pid.maxIAccum, 0);
    //         case 3:
    //             // pidController.setP(pid.p);
    //             // pidController.setI(pid.i);
    //             // pidController.setD(pid.d);
    //             Sparkconfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).
    //             pid(pid.p, pid.i, pid.d);
    //         break;
    //     }
    // }

}
