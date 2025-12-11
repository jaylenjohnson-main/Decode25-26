package org.firstinspires.ftc.teamcode.mechanisms;
//can you see changes?
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    DcMotor intake = null;
    public Intake(HardwareMap hwMap){
        intake = hwMap.get(DcMotor.class, "intake");
    }

    public void on(){
        intake.setPower(1.0);

    }

    public void off(){
        intake.setPower(0);

    }


}
