package org.firstinspires.ftc.teamcode.Utilities;

public class xCusedPIDController extends xCusedPIDFController {

    /**
     * Default constructor with just the coefficients
     */
    public xCusedPIDController(double kp, double ki, double kd) {

        super(kp, ki, kd, 0);

    }

    /**
     * The extended constructor.
     */
    public xCusedPIDController(double kp, double ki, double kd, double sp, double pv) {

        super(kp, ki, kd, 0, sp, pv);

    }

    public void setPID(double kp, double ki, double kd) {

        setPIDF(kp, ki, kd, 0);

    }

}
