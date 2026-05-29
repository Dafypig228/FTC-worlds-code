package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.util.Range;


public class LerpController {
    xCusedTimer timer = new xCusedTimer();

    boolean isBusy = false;
    double startPos = 0;
    double endPos = 0;
    double duration = 0;
    boolean cutEdges = false;

    public void beginMovement(double startPos, double endPos, double duration, boolean cutEdges){
        if (startPos == endPos)
            return;
        this.isBusy = true;
        this.cutEdges = true;
        this.startPos = startPos;
        this.endPos = endPos;
        this.duration = duration;
        timer.reset();
    }

    public double getPosition(){
        if (timer.milliseconds() >= duration)
            isBusy = false;
        return calculateProgress(Range.clip(timer.milliseconds() / duration, 0, 1)) * (endPos - startPos) + startPos;
    }

    public double calculateProgress(double x){
        if (cutEdges){
            x = (x - 0.5) * 0.8 + 0.5;
        }
        return -(Math.cos(Math.PI * x) - 1) / 2;
    }

    public double getEndPos(){
        return endPos;
    }

    public void interrupt(){
        isBusy = false;
    }

    public boolean isBusy(){
        return isBusy && timer.milliseconds() < duration;
    }
}
