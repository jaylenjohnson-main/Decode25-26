package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;



public class MecanumDrive {

    //Note: we are using the SparkFunOdometry Sensor aka) https://www.sparkfun.com/sparkfun-optical-tracking-odometry-sensor-paa5160e1-qwiic.html

    //Motors for 4wd
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private SparkFunOTOS otos = null;

    private SparkFunOTOS.Pose2D pos;
    private IMU imu = null;
    private Servo light;

    private double absoluteValueLA = 0;
    private double LA = absoluteValueLA;

    //Robot Speed Tuning
    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
  /*  final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   =  0.010  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)


    final double MAX_AUTO_SPEED = 0.1;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.1;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.1;   //  Clip the turn speed to this max value (adjust for your robot)
    private double headingError  = 0;
    
   */


    public MecanumDrive(HardwareMap hardwareMap){

        //init Motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        //frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //  backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        imu = hardwareMap.get(IMU.class, "imu");
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");


        light = hardwareMap.get(Servo.class,"light");
    }

    public void slowTurn(double rotate){setPowers(rotate, -rotate,rotate,-rotate);}

    //helper method private to this class to ensure motor pis never set higher than one
    private void setPowers(double fl, double fr, double bl, double br){
        double maxSpeed = 0.0;
        maxSpeed = Math.max(maxSpeed, Math.abs(fl));
        maxSpeed = Math.max(maxSpeed, Math.abs(fr));
        maxSpeed = Math.max(maxSpeed, Math.abs(bl));
        maxSpeed = Math.max(maxSpeed, Math.abs(br));

        fl /= maxSpeed;
        fr /= maxSpeed;
        bl /= maxSpeed;
        br /= maxSpeed;

        frontLeft.setPower(fl/2);
        frontRight.setPower(fr/2);
        backLeft.setPower(bl/2);
        backRight.setPower(br/2);


    }
    //public method to call to drive the chassis
    public  void drive(double forward, double right, double rotate){

        double fl = forward + right + rotate;
        double fr = forward - right - rotate;
        double bl = forward - right + rotate;
        double br = forward + right - rotate;

        setPowers(fl ,fr ,bl, br);
    }



    //method to stop the robot
    public void stopMoving() {

            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
    }
}
 

