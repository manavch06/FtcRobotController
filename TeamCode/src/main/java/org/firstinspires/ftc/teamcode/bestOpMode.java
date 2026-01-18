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
 *//* Copyright (c) 2017 FIRST. All rights reserved.
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

@TeleOp(name="I don't know. What should I call the teleop? Be Like Ryke. I don't know. I think our teleop should be um...Something German. Let me google it...Something German...Not helpful Not helpful. Ok, wait, wait, wait. Sid Meier's Civilization VIII. And then what am I gonna do. But now I can do it all in my head. Journey Before Destination. Be Like Ryke. Be Like Ryke. Be Like Ryke. Be Like Ryke. BE LIKE RYKE! BE LIKE RYKE! BE LIKE RYKE! BE LIKE RYKE! BE LIKE RYKE! Teleop", group="Iterative OpMode")
public class bestOpMode extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    driveMech drive = new driveMech();
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 1.75 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH_LINEAR  = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double COUNTS_PER_INCH_CLAW = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION);
    private DcMotor launcher;
    private DcMotor launcher2;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        drive.init(hardwareMap);
        launcher = hardwareMap.dcMotor.get("launcher");
        launcher2 = hardwareMap.dcMotor.get("launcher21111");

        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        launcher.setDirection(DcMotor.Direction.REVERSE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        telemetry.addData("Status", "Started");
        telemetry.addData("Front Left Power:", drive.getPowerFrontLeft());
        telemetry.addData("Front Right Power:", drive.getPowerFrontRight());
        telemetry.addData("Back Left Power:", drive.getPowerBackLeft());
        telemetry.addData("Back Right Power:", drive.getPowerBackRight());
        runtime.reset();
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Running");
        telemetry.addData("Front Left Power:", drive.getPowerFrontLeft());
        telemetry.addData("Front Right Power:", drive.getPowerFrontRight());
        telemetry.addData("Back Left Power:", drive.getPowerBackLeft());
        telemetry.addData("Back Right Power:", drive.getPowerBackRight());

        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);

        drive.drive(theta, power, turn);

        boolean a = gamepad2.a;
        if (a) {
            int pos = launcher.getCurrentPosition();
            launcher.setPower(0.15);
            launcher.setTargetPosition(pos + 1000);
            launcher.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (launcher.isBusy()) {
                telemetry.addData("Path", "Processing");
            }
            telemetry.addData("Path", "Complete");
            launcher.setPower(0);
            launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // reset the timeout time and start motion.
        runtime.reset();
    }

}
