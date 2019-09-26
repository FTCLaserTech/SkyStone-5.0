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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DigitalChannel;
//import com.qualcomm.robotcore.hardware.PIDCoefficients;
//import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;

import java.util.ArrayList;
import java.util.Arrays;

//import static java.lang.Math.abs;

//This is the hardware map for our Mechanum robot. It has 4 Dc motors established.

public class Hardware_Map
{
    // IMU variables
    public BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    public Orientation lastAngles = new Orientation();
    public double globalAngle, power = .30, correction;

    // Declare OpMode members.
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    public ColorSensor colorSensor;
    public DistanceSensor distanceSensor;
    public BNO055IMU imu;


    static final double     COUNTS_PER_MOTOR_REV    = 28;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 50.9 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 96/25.4 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.141529);

    // Variables for power
    double frontLeftPower;
    double frontRightPower;
    double backLeftPower;
    double backRightPower;

    public Hardware_Map(LinearOpMode linearOpMode)
    {

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;  //COULD DEGRESS OR RADIANS BE THE PROBLEM WITH THE ROTATION MATRIX
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // reset IMU
        resetHeading(linearOpMode);



        // Set up the 4 motors so that they are recognized by the program
        frontLeft = linearOpMode.hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = linearOpMode.hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = linearOpMode.hardwareMap.get(DcMotor.class, "backLeft");
        backRight = linearOpMode.hardwareMap.get(DcMotor.class, "backRight");

        // Most robots need the motor(s) on one side to be reversed to drive forward
        // Reverse the motor(s) that run(s) backwards when connected directly to the battery
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Set the behavior of each motor when it's not moving so that it actively brakes instead of just drifting
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        colorSensor = linearOpMode.hardwareMap.colorSensor.get("color_sensor");
        distanceSensor = linearOpMode.hardwareMap.get(DistanceSensor.class,"range_sensor");

    }

    public void resetHeading(LinearOpMode linearOpMode)
    {
        imu = linearOpMode.hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        linearOpMode.telemetry.addData("Mode", "calibrating...");
        linearOpMode.telemetry.update();

        while (!linearOpMode.isStopRequested() && !imu.isGyroCalibrated()) {
            linearOpMode.sleep(50);
            linearOpMode.idle();
        }
        linearOpMode.telemetry.addData("Mode", "waiting for start");
        linearOpMode.telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        linearOpMode.telemetry.update();

    }
    // STRAFING METHODS

    public void Forward(double power)
    {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    public void Backward(double power)
    {
        frontLeft.setPower(-power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(-power);
    }

    public void Left(double power)
    {
        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(-power);
    }

    public void Right(double power)
    {
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(power);
    }



    // DIAGONAL METHODS

    public void ForwardLeft(double power)
    {
        frontRight.setPower(power);
        backLeft.setPower(power);
    }

    public void ForwardRight(double power)
    {
        frontLeft.setPower(power);
        backRight.setPower(power);
    }

    public void BackwardLeft(double power)
    {
        frontLeft.setPower(-power);
        backRight.setPower(-power);
    }

    public void BackwardRight(double power)
    {
        frontRight.setPower(-power);
        backLeft.setPower(-power);
    }

    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        //double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        //if (deltaAngle < -180)
        //    deltaAngle += 360;
        //else if (deltaAngle > 180)
        //    deltaAngle -= 360;

        //globalAngle += deltaAngle;

        //lastAngles = angles;

        //return globalAngle;
        return angles.firstAngle;
    }
    // Change average for how many readings we want

    public double colorSensorRedAverage()
    {
        int samples = 10;
        double average = 0;

        for(int i =samples; i>0; i--)
        {
            average += colorSensor.red();
        }
        average = average/samples;
        return average;
    }
    public double colorSensorBlueAverage()
    {
        int samples = 10;
        double average = 0;
        for(int i =samples; i>0; i--)
        {
            average += colorSensor.blue();
        }
        average = average/samples;
        return average;
    }
    public double colorSensorGreenAverage()
    {
        int samples = 10;
        double average = 0;

        for(int i =samples; i>0; i--)
        {
            average += colorSensor.green();
        }
        average = average/samples;
        return average;
    }
    public double distanceSensorAverage()
    {
        int samples = 10;
        double average = 0;
        for(int i =samples; i>0; i--)
        {
            average += distanceSensor.getDistance(DistanceUnit.CM);
        }
        average = average/samples;
        return average;
    }
}
