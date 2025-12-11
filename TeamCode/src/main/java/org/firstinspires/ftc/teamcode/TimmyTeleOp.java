package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
//import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp()
public class TimmyTeleOp extends OpMode{

    //MecanumDrive cosmo = null;
    Intake vicky = null;
    Launcher vector = null;
    private MecanumDrive cosmo;

    @Override
    public void init() {
        cosmo = new MecanumDrive(hardwareMap);
        vicky = new Intake(hardwareMap);
        vector = new Launcher(hardwareMap);
    }

    @Override
    public void loop() {
        //code to make robot drive, strafe and turn
        cosmo.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        //cosmo.setLightColor();

        //intake code
        if (gamepad2.a) {
            vicky.on();
        } else if (gamepad2.b) {
            vicky.off();
        }
        if (gamepad2.x) {
            vector.up();
        } else if (gamepad2.y) {
            vector.down();
        }

        //outtake code
        if (gamepad2.right_bumper) {
            vector.on();
        } else if (gamepad2.left_bumper) {
            vector.off();
        }
        if (gamepad1.right_bumper) {
            cosmo.slowTurn(0.5);
        } else if (gamepad1.left_bumper) {
            cosmo.slowTurn(-0.5);
        }


        //telemetry.addData("X Position", cosmo.getX());
        //telemetry.addData("Y Position", cosmo.getY());
        //telemetry.addData("Heading", cosmo.getH());
    }
    }