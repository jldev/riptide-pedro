package org.firstinspires.ftc.teamcode.riptide.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PIDMotor extends MotorEx {
    public PIDMotor(@NonNull HardwareMap hMap, String id, double cpr, double rpm) {
        super(hMap, id, cpr, rpm);

    }

    public void setPIDCoefficients(double kp, double ki, double kd){
        positionController.setPID(kp, ki, kd);
    }

}
