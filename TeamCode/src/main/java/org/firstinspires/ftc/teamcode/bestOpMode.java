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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.driveMech;

@TeleOp(name="Ryken Teleop", group="Iterative OpMode")
public class bestOpMode extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor linearActuator;
    //private DcMotor claw;
    driveMech drive = new driveMech();
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 1.4 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH_LINEAR  = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double COUNTS_PER_INCH_CLAW = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION);

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        drive.init(hardwareMap);
        telemetry.addData("Status", "Initialized");

        linearActuator  = hardwareMap.dcMotor.get("linearActuator");
        linearActuator.setDirection(DcMotor.Direction.REVERSE);
        //linearActuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //claw = hardwareMap.dcMotor.get("claw");
        //claw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //claw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void start() {
        telemetry.addData("Status", "Started");
        runtime.reset();
    }

    @Override
    public void loop() {
        double forwardBackward = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        boolean actuatorUp = gamepad1.dpad_up;
        boolean actuatorDown = gamepad1.dpad_down;
        boolean actuatorRight = gamepad1.dpad_right;
        boolean clawOpen = gamepad1.a;
        boolean clawClose = gamepad1.b;

        drive.drive(forwardBackward, turn);

        if (actuatorUp) {
            linearActuator.setTargetPosition((int) (28.0 * COUNTS_PER_INCH_LINEAR)); //Height of lower net is 25.75 inches
            linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearActuator.setPower(-0.5);
            linearActuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            while (linearActuator.isBusy()) {
                telemetry.addData("Status", "linearActuator moving");
            }
        } else if(actuatorRight) {
            linearActuator.setTargetPosition((int) (47.0 * COUNTS_PER_INCH_LINEAR)); //Height of upper net is 43 inches
            linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearActuator.setPower(-0.5);
            linearActuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            while (linearActuator.isBusy()) {
                telemetry.addData("Status", "linearActuator moving");
            }
        } else if (actuatorDown) {
            linearActuator.setTargetPosition(0);
            linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearActuator.setPower(-0.1);
            while (linearActuator.isBusy()) {
                telemetry.addData("Status", "linearActuator moving");
            }
        } else {
            linearActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            linearActuator.setPower(0);
        }

        // Turn On RUN_TO_POSITION

        // reset the timeout time and start motion.
        runtime.reset();
        //if (clawOpen) {
            //claw.setTargetPosition((int) (.1 * COUNTS_PER_INCH_CLAW));
            //claw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //claw.setPower(0.5);
            //claw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //while (claw.isBusy()) {
                //telemetry.addData("Status", "claw opening");
            //}
        //} else if (clawClose) {
            //claw.setTargetPosition(0);
            //claw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //claw.setPower(0.5);
            //claw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //while (claw.isBusy()) {
                //telemetry.addData("Status", "claw closing");
            //}
        //}
    }

    @Override
    public void stop() {
        telemetry.addData("Status", "Stopped");
    }

}
