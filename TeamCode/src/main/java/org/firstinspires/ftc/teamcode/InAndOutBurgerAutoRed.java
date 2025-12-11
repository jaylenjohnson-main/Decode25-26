/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;


@Autonomous
public class InAndOutBurgerAutoRed extends LinearOpMode {

    //declare mechanisms
    MecanumDrive cosmo = null;
    Intake intake = null;
    Launcher vector = null;
    Launcher frontFlipper = null;


    //declare timer
    ElapsedTime runtime = null;


    @Override
    public void runOpMode() throws InterruptedException {

        cosmo = new MecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap);
        vector = new Launcher(hardwareMap);
        frontFlipper = new Launcher(hardwareMap);

        runtime = new ElapsedTime();

        waitForStart();
        runtime.reset();
        //step 1: drives backwards for 1.5 second
        while (opModeIsActive() && runtime.milliseconds() < 1050) {
            cosmo.drive(0.19, 0, 0);
        }

        cosmo.stopMoving();
        runtime.reset();

        //turns on outtake
        while (opModeIsActive() && runtime.milliseconds() < 5000) {
            vector.on();
        }

        runtime.reset();

        //moves up flipper
        while (opModeIsActive() && runtime.milliseconds() < 2000) {

            frontFlipper.down();
        }
        runtime.reset();

        //moves down flipper
        while (opModeIsActive() && runtime.milliseconds() < 2000) {
            frontFlipper.up();
        }

        //wait(1000);
        runtime.reset();

        //turn on intake
        while (opModeIsActive() && runtime.milliseconds() < 2000) {
            intake.on();
        }

        runtime.reset();
        //moves up flipper
        while (opModeIsActive() && runtime.milliseconds() < 1000) {
            frontFlipper.down();
        }
        runtime.reset();
        //moves down flipper
        while (opModeIsActive() && runtime.milliseconds() < 2000) {
            frontFlipper.up();

        }
        runtime.reset();
        while (opModeIsActive() && runtime.milliseconds() < 1000) {
            frontFlipper.down();
        }
        runtime.reset();
        //moves down flipper
        while (opModeIsActive() && runtime.milliseconds() < 2000) {
            frontFlipper.up();

        }
        runtime.reset();

        while (opModeIsActive() && runtime.milliseconds() < 700){
            cosmo.drive(0,0.1,0);
        }
        cosmo.stopMoving();
        runtime.reset();

    }
}
