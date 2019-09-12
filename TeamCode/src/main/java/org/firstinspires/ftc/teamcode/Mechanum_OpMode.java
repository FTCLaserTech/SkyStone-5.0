/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;

import java.util.ArrayList;
import java.util.Arrays;

@TeleOp(name="Mechanum_OpMode", group="Linear Opmode")
public class Mechanum_OpMode extends LinearOpMode {



    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;


    // Declare OpMode members.
    private Hardware_Map hwmap;

    @Override
    public void runOpMode() {

        hwmap = new Hardware_Map(this);

        // Sets the adjustable speed variable
        double speed = 8;

        waitForStart();
        hwmap.runtime.reset();



        //speed = speed - 0.1
        while (opModeIsActive())
        {


            if(gamepad1.dpad_down == true) {
                speed -= 0.1; }

            if(gamepad1.dpad_up == true) {
                speed += 0.1; }

            if(gamepad1.left_bumper == true) {
                speed = 0.1;  }

            if(gamepad1.right_bumper == true) {
                speed = 1;  }



            // Reading joystick imputs
            hwmap.stick_forward = gamepad1.left_stick_y;
            hwmap.stick_sideways = gamepad1.left_stick_x;
            hwmap.stick_rotation = gamepad1.right_stick_x;


            hwmap.frontLeftPower = hwmap.stick_forward - hwmap.stick_sideways + hwmap.stick_rotation;
            hwmap.frontRightPower = hwmap.stick_forward + hwmap.stick_sideways - hwmap.stick_rotation;
            hwmap.backLeftPower = hwmap.stick_forward + hwmap.stick_sideways + hwmap.stick_rotation;
            hwmap.backRightPower = hwmap.stick_forward - hwmap.stick_sideways - hwmap.stick_rotation;


            ArrayList<Double> motorList = new ArrayList<>(Arrays.asList(hwmap.frontLeftPower, hwmap.frontRightPower, hwmap.backLeftPower, hwmap.backRightPower));


            double highest = 0;

            // Find the highest value out of the 4 motor values
            for (double n : motorList) {
                if (Math.abs(n) > highest)  {
                    highest = Math.abs(n);
                }
            }


            // Normalize each of the values
            hwmap.frontLeftPower = (hwmap.frontLeftPower / highest) * (speed / 10);
            hwmap.frontRightPower = (hwmap.frontRightPower / highest) * (speed / 10);
            hwmap.backLeftPower = (hwmap.backLeftPower / highest) * (speed / 10);
            hwmap.backRightPower = (hwmap.backRightPower / highest) * (speed / 10);

            hwmap.frontLeft.setPower(hwmap.frontLeftPower);
            hwmap.frontRight.setPower(hwmap.frontRightPower);
            hwmap.backLeft.setPower(hwmap.backLeftPower);
            hwmap.backRight.setPower(hwmap.backRightPower);

            // Show the elapsed game time and wheel power (only works with code method two)
            telemetry.addData("Status", "Run Time: " + hwmap.runtime.toString());
            telemetry.addData("Wheel Power", "left (%.2f), right (%.2f)", hwmap.frontLeftPower, hwmap.frontRightPower, hwmap.backLeftPower, hwmap.backRightPower);
            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.update();
        }
    }
}
