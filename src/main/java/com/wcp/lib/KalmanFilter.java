package com.wcp.lib;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class KalmanFilter {
    int limit =10;
    double[] Xkp = new double[2];
    double[] Pkp = new double[2];

    double[] K = new double[2];
    double[] Xk = new double[2];

    double[] Xkz = new double[2];
    double[] Pkz = new double[2];

    double[] Qk = {0.0000000000000001,0};//error per meter of odo
    List<double[]> Ykh = new ArrayList<double[]>();
    //z is a placeholder for last updates version of that var
    // make init procces covarriance matrix : just zeros, should be init variance but shouldnt matter idk we will see
    /* 1.The predicted state: Xkp = Xkz + Uk where Uk is delta pose from odo
     * 2.The predicted Prccess covariance matriX: Pkp = Pkz + Qk )This is error expected for each tick of odo running
     * 3.Kalman Gain: K = pkp/(pkp+R) where R is the observed variance (std^2)
     * 4.Calculate current state: Xk = Xkp + K(Yk-Xkp) where Yk is the current observation
     */
    public KalmanFilter(){
}
 
    public void update(double[] deltaPose, double[] Yk){
    Ykh.add(Yk);
    for(int i = 0; i<2; i++){
        Xkp[i] = (Xkz[i]) + (deltaPose[i]);
        Pkp[i] = Pkz[i] + deltaPose[i]*Qk[i];
        K[i] = Pkp[i]/(Pkp[i]+getR()[i]);
        Xk[i] = Xkp[i] + K[i]*(Yk[i] - Xkp[i]);
    }
    Pkz =Pkp.clone();
    Xkz =Xk.clone();

    }
    public double[] getR(){
        
        double[] var = new double[2];
        for(int j = 0; j<2; j++){
            double[] mean = new double[2];
            for(int i =0; i<limit&&i<Ykh.size()-1;i++){
                mean[j] += Ykh.get(j)[0];
            }
            mean[j]/=Ykh.size();

            for(int i =0; i<limit&&i<Ykh.size()-1;i++){
                var[j] += (Ykh.get(i)[j]-mean[j])*(Ykh.get(i)[j]-mean[j]);
            }
            var[j]/=Ykh.size();
        
        }

        return var;
    }
}
