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

package org.firstinspires.ftc.teamcode.section1;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.I2cAddressableDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Basic: League Test Drive", group = "Iterative Opmode")
public class BasicTestLeagueThing extends OpMode {

    // Declare OpMode members.
    private DcMotor leftFront = null;   // white - port 2
    private DcMotor leftRear = null;    // yellow - port 1
    private DcMotor rightFront = null;  // green - port 3
    private DcMotor rightRear = null;   // blue - port 0

    private Servo clawServo = null;
    private CRServo armMotor = null;
    private DistanceSensor sensorRange = null;

    private DigitalChannel armStop = null;

    private I2cAddressableDevice armEncoder = null;

    private double clawPos;
    private final double clawIncrement = 0.02;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");


        // servos
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        armMotor = hardwareMap.get(CRServo.class, "armMotor");

        // clawPos = clawServo.getPosition();
        clawPos = 0.5;

        // arm touch sensor
        armStop = hardwareMap.get(DigitalChannel.class, "armStop");
        armStop.setMode(DigitalChannel.Mode.INPUT);

        // REV Robotics IR 2m Sensor
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        // Trying to connect to arm motor's I2C encoder
        armEncoder = hardwareMap.get(I2cAddressableDevice.class, "armEncoder");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        clawServo.setDirection(Servo.Direction.FORWARD);
        armMotor.setDirection(CRServo.Direction.FORWARD);

        // while the power of the wheels is 0, brake
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Claw position", "%4.2f", clawPos);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        clawServo.setPosition(clawPos);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    @Override
    public void loop() {
        // Setup a variable for each side of the robot
        double leftPower;
        double rightPower;
        double targetRange;
        final double MAX_RANGE = 36.0;
        final double MID_SPEED = 0.25;
        final double MID_RANGE = 12.0;
        final double MIN_SPEED = 0.125;
        final double MIN_RANGE = 1.0;

        // ***
        byte armPositionByte;
        int armPosition;

        // *** Let's see if we can read the encoder value from arm motor
        telemetry.addData("Arm's I2C Address: ", armEncoder.getI2cAddress().get8Bit());
        telemetry.addData("Left Rear's conncection info: ", leftRear.getController().getConnectionInfo());
        // armPositionByte = armEncoder.(0x4c);
        // armPosition = TypeConversion.unsignedByteToInt(armPositionByte);
        // telemetry.addData("Arm's position: ", "%4d", armPosition);

        // This driving mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.

        // strafing control

        double control = 2;

        if (gamepad1.right_bumper) {
            control = 4;
        } else if (gamepad1.left_bumper) {
            control = 1;
        }

        double driveY = -gamepad1.left_stick_y;
        double driveX = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        // Get the range to target using the distance sensor.  Allow movement forward only if there
        // is no obstacles with 1ft of the robot
        targetRange = sensorRange.getDistance(DistanceUnit.INCH);
        telemetry.addData("Distance to obstacle: ", "%4.2f", targetRange);
        if (driveY > 0){
            if (targetRange <= MIN_RANGE){
                driveY = 0;
            }
            else if (targetRange <= MID_RANGE){
                driveY = MIN_SPEED;
            }
            else if (targetRange <= MAX_RANGE){
                driveY = MID_SPEED;
            }
        }

        // ^ condensed
        double leftFrontPower = Range.clip((driveY + driveX) + turn, -1.0, 1.0);
        double leftRearPower = Range.clip((driveY - driveX) + turn, -1.0, 1.0);
        double rightFrontPower = Range.clip((driveY - driveX) - turn, -1.0, 1.0);
        double rightRearPower = Range.clip((driveY + driveX) - turn, -1.0, 1.0);

        leftFront.setPower(leftFrontPower / control);
        leftRear.setPower(leftRearPower / control);
        rightFront.setPower(rightFrontPower / control);
        rightRear.setPower(rightRearPower / control);

        // controlling the claw

        if (gamepad1.dpad_right) {
            clawPos += clawIncrement;
        } else if (gamepad1.dpad_left) {
            clawPos -= clawIncrement;
        }
        clawPos = Range.clip(clawPos, 0.0, 1.0);
        clawServo.setPosition(clawPos);
        telemetry.addData("Claw position", "%4.2f", clawPos);

        // control arm
        if (gamepad1.dpad_up) {
            armMotor.setPower(0.5);
        } else if (gamepad1.dpad_down && armStop.getState()) {
            armMotor.setPower(-0.5);
        } else {
            armMotor.setPower(0);
        }


        // write touch sensor control to screen
        telemetry.addData("Touch Sensor", armStop.getState());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // stop all motion
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);

        // Return claw to normal position
        clawServo.setPosition(0.5);

    }

}
