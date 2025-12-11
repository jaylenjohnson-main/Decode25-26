package org.firstinspires.ftc.teamcode.mechanisms;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Launcher {
    Servo frontFlipper = null;
    //Servo backFlipper = null;
    DcMotor outtake = null;
    CRServo conveyor = null;

public Launcher(HardwareMap hwMap) {
    frontFlipper = hwMap.get(Servo.class, "frontFlipper");
    outtake = hwMap.get(DcMotor.class, "outtake");
    outtake.setDirection(DcMotor.Direction.REVERSE);
}

    public void up(){
        frontFlipper.setPosition(0);
        //backFlipper.setPosition(0);
    }

    public void down(){
        frontFlipper.setPosition(0.4);
        //backFlipper.setPosition(0.5);
    }
  
    public void on(){
        outtake.setPower(0.75);
    }
    public void  off(){
        outtake.setPower((0.0));
    }
    //public void conveyorOn(double v){
        //conveyor.setPower(1);
    }
    //public void conveyorOff(int i){
       // conveyor.setPower(0);