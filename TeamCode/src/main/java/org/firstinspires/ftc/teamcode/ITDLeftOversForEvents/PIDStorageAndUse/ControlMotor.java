package org.firstinspires.ftc.teamcode.ITDLeftOversForEvents.PIDStorageAndUse;
import com.qualcomm.robotcore.util.ElapsedTime;
@com.acmerobotics.dashboard.config.Config
public class ControlMotor {
    public static double integralSum =0;
    public static double kpIntake =0.0090;
    public static double kdIntake =0.00018;

    public static double kpUppy=0.0030;
    public static double kdUppy=0.0001;
    ElapsedTime timer=new ElapsedTime();

    public double getLastError() {
        return lastError;
    }

    private double lastError=0;

    public double PIDControl(double targetPosition, double curentPosition){
        double error= targetPosition - curentPosition;
        double derivative=(error-lastError) / timer.seconds();

        lastError = error;
        timer.reset();
        double pid = (error* kpIntake +derivative* kdIntake);
        if(pid<0) pid*=1.2;
        if(pid<0 && curentPosition<5) pid *= 0.3;
        else if(pid<0 && curentPosition<20) pid = pid*0.7;
        else if(pid<0 && curentPosition<40) pid = pid*1.3;
        return pid;
    }


    public double PIDControlUppy(double targetPosition, double curentPosition){
        double error= targetPosition - curentPosition;
        double derivative=(error-lastError) / timer.seconds();

        lastError = error;
        timer.reset();
        double pid = error*kpUppy+derivative*kdUppy;

        if(targetPosition==0 && curentPosition >= -350) pid *=1.1;
        if(targetPosition > -900 && curentPosition >= -800) pid*=1.6;
        if(curentPosition > -20 && targetPosition==0) pid = 0;
        if(curentPosition > -200 && targetPosition==0) pid *= 2;
        if(targetPosition < -2300 && curentPosition >=-200) pid*=2;
        return pid;

    }
}


