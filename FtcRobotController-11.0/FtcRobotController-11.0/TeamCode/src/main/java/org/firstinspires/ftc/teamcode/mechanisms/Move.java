package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Move {
    DcMotor Move = null;
    public Move(HardwareMap hwMap){
        Move = hwMap.get(DcMotor.class, "backLeft");
        Move = hwMap.get(DcMotor.class, "backRight");
        Move = hwMap.get(DcMotor.class, "frontLeft");
        Move = hwMap.get(DcMotor.class, "frontRight");
        Move.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void on(){
        Move.setPower(0.35);

    }

    public void off(){
        Move.setPower(0);

    }


}
