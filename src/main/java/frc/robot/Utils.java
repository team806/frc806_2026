package frc.robot;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.REVLibError;

public class Utils {
    public static boolean IsDoubleApproximately(double left, double right, double sigma){
        return left >= right - sigma && left <= right + sigma;
    }

    public static boolean RevMotorIsConnected(SparkBase motor) {
        return motor.getLastError() != REVLibError.kCANDisconnected;
    }
}
