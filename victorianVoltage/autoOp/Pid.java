package org.firstinspires.ftc.teamcode.opModes.victorianVoltage.autoOp;

public class Pid {
    private double driveP;     // Tuning variable for PID.
    private double driveI;   // Eliminate integral error in 1 sec.
    private double driveD;   // Account for error in 0.1 sec.
    // Protect against integral windup by limiting integral term.
    private final double drivePidIntMax = 1;  // Limit to max speed.
    private final double driveOutMax = 1.0;  // Motor output limited to 100%.
    private double PVal = 0;
    private double IVal = 0;
    private double DVal = 0;
    private static double previousError = 0;

    public Pid(double p, double i, double d){
        driveP = p;
        driveI = i;
        driveD = d;
    }

    public double controlOutput(double targetValue, double realValue,double deltaT) {
        double error = targetValue - realValue;
        PVal = error * driveP;
        IVal = (IVal + error*deltaT) * driveI;
        DVal = ((error - previousError)/deltaT) * driveD;
        previousError = error;


        System.out.println("Prop" + " "+ PVal);
        System.out.println("Integral" + " "+ IVal);
        System.out.println("Deriv" + " " + DVal);

        return PVal + IVal + DVal;
    }


}
