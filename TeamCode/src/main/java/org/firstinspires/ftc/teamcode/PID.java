package org.firstinspires.ftc.teamcode;

import java.util.Date;
import java.util.Timer;


public class PID
{
    double imin, imax, omin, omax;
    protected double set, process, error, KP, KI, KD, totalError, lastError, P, I, D, output;
    protected long uPeriod;
    Date date = new Date();
    long now, lastTime;

    public PID(double KP, double KI, double KD, double imax, double imin, double omax, double omin, long uPeriod)
    {
        this.imin = imin;
        this.imax = imax;
        this.omin = omin;
        this.omax = omax;

        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
        this.totalError = 0;
        this.lastError = 0;
        this.P = 0;
        this.I = 0;
        this.D = 0;
        this.output = 0;

        this.now = 0;
        this.lastTime =0;

        this.uPeriod = uPeriod;

        PIDThread thread = new PIDThread();
        thread.start();
    }

    public synchronized void setPoint(double set)
    {
        this.set = set;
    }

    public synchronized void setProcess(double process)
    {
        this.process = process;
    }

    public synchronized double getOutput()
    {
        return output;
    }

    public synchronized double getP()
    {
        return P;
    }

    public synchronized double getI()
    {
        return I;
    }

    public synchronized double getD()
    {
        return D;
    }

    class PIDThread extends Thread
    {
        public synchronized void run()
        {
            lastTime = date.getTime();
            while (true)
            {
                try
                {
                    now = date.getTime();
                    double timeChange = (double)(now - lastTime);

                    Thread.sleep(uPeriod);

                    error = set - process;
                    P = KP * error;

                    totalError += error * timeChange;
                    I = KI * totalError;
                    if(I > imax)
                        I = imax;
                    if(I < imin)
                        I = imin;

                    if (timeChange != 0.0) {
                        D = ((error - lastError) * KD) / timeChange;
                    }
                    D = 0;

                    output = P + I + D;

                    if(output > omax)
                        output = omax;
                    else if (output < omin)
                        output = omin;

                    lastError = error;

                    lastTime = now;
                }
                catch (Exception e){System.out.println(e.toString());}
            }
        }
    }
}
