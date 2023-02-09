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

package org.firstinspires.ftc.teamcode.drive.opmode.main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Main Test")
public class MainTest extends LinearOpMode {

    Servo sExtend1, sExtend2, sUpDownClaw1, sUpDownClaw2, sRotateClaw, sClaw;
    DcMotor motorLift0, motorLift1;
    DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;
    double inr = 0.7;

    @Override
    public void runOpMode() {
        sExtend1 = hardwareMap.servo.get("serv0");
        sExtend2 = hardwareMap.servo.get("serv1");
        sUpDownClaw1 = hardwareMap.servo.get("serv2");
        sUpDownClaw2 = hardwareMap.servo.get("serv3");
        sRotateClaw = hardwareMap.servo.get("serv4");
        sClaw = hardwareMap.servo.get("serv5");
        motorLift0 = hardwareMap.dcMotor.get("motor0");
        motorLift1 = hardwareMap.dcMotor.get("emotor1");
        motorFrontLeft = hardwareMap.dcMotor.get("motor1");
        motorFrontRight = hardwareMap.dcMotor.get("motor3");
        motorBackLeft = hardwareMap.dcMotor.get("emotor2");
        motorBackRight = hardwareMap.dcMotor.get("emotor3");
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //sExtend1.scaleRange(0, 0.965);
        waitForStart();
        while (opModeIsActive()) {
            double y = -gamepad1.right_stick_y; // Remember, this is reversed!
            double y1 = -gamepad2.right_stick_y * 0.2;
            double x = gamepad1.right_stick_x * 1.1; // Counteract imperfect strafing
            double x1 = gamepad2.right_stick_x * 0.22;
            double rx = gamepad1.left_stick_x;
            double rx1 = gamepad2.left_stick_x * 0.2;
            double denominator = Math.max(Math.abs(y) + Math.abs(y1) + Math.abs(x) + Math.abs(x1) + Math.abs(rx) + Math.abs(rx1), 1);
            double frontLeftPower = (y + y1 + x + x1 + rx + rx1) / denominator;
            double backLeftPower = (y + y1 - x - x1 + rx + rx1) / denominator;
            double frontRightPower = (y  + y1 - x - x1 - rx - rx1) / denominator;
            double backRightPower = (y + y1 + x + x1 - rx - rx1) / denominator;
            if(frontLeftPower >= 0)
                motorFrontLeft.setPower(inr * Math.sqrt(Math.abs(frontLeftPower)));
            else
                motorFrontLeft.setPower(-inr * Math.sqrt(Math.abs(frontLeftPower)));
            if(backLeftPower >= 0)
                motorBackLeft.setPower(inr * Math.sqrt(Math.abs(backLeftPower)));
            else
                motorBackLeft.setPower(-inr * Math.sqrt(Math.abs(backLeftPower)));
            if(frontRightPower >= 0)
                motorFrontRight.setPower(inr * Math.sqrt(Math.abs(frontRightPower)));
            else
                motorFrontRight.setPower(-inr * Math.sqrt(Math.abs(frontRightPower)));
            if(backRightPower >= 0)
                motorBackRight.setPower(inr * Math.sqrt(Math.abs(backRightPower)));
            else
                motorBackRight.setPower(-inr * Math.sqrt(Math.abs(backRightPower)));

            if(gamepad2.right_bumper) {
                sExtend1.setPosition(sExtend1.getPosition() + 0.003);
                sExtend2.setPosition(sExtend2.getPosition() - 0.003);
            } else if(gamepad2.left_bumper) {
                sExtend1.setPosition(sExtend1.getPosition() - 0.003);
                sExtend2.setPosition(sExtend2.getPosition() + 0.003);
            }
            if(gamepad2.dpad_down){
                sUpDownClaw1.setPosition(sUpDownClaw1.getPosition() + 0.003);
                sUpDownClaw2.setPosition(sUpDownClaw2.getPosition() + 0.003);
            } else if(gamepad2.dpad_up){
                sUpDownClaw1.setPosition(sUpDownClaw1.getPosition() - 0.003);
                sUpDownClaw2.setPosition(sUpDownClaw2.getPosition() - 0.003);
            }
            if(gamepad2.b){
                sRotateClaw.setPosition(0);
            } else if(gamepad2.x){
                sRotateClaw.setPosition(1);
            }
            if(gamepad2.a){
                sClaw.setPosition(0);
            } else if(gamepad2.y){
                sClaw.setPosition(0.5);
            }
            if(gamepad2.dpad_left){
                sExtend1.setPosition(1);
                sExtend2.setPosition(0);
            } else if(gamepad2.dpad_right){
                sExtend1.setPosition(0);
                sExtend2.setPosition(1);
            }
            if(gamepad2.right_stick_button){
                PUTCONEIntakePosition();
                IDLEIntakePosition();
            }
            if(gamepad1.a){
                IDLEIntakePosition();
                GRABIntakePosition(0.87, 0);
                PUTCONEIntakePosition();
//                GRABIntakePosition(0.83, 0);
//                PUTCONEIntakePosition();
//                GRABIntakePosition(0.87, 0);
//                PUTCONEIntakePosition();
                IDLEIntakePosition();
            }
            motorLift0.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            motorLift1.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            telemetry.addLine("Position Extend:" + sExtend1.getPosition());
            telemetry.addLine("Position Up/Down: " + sUpDownClaw1.getPosition());
            telemetry.update();
        }
    }

    /**
     * @param position position of the extension of intake
     *                 Position: 1, fully folded(to the robot)
     *                 Position: 0, fully extended
     */
    protected void setExtendIntake(double position){
        sExtend1.setPosition(position);
        sExtend2.setPosition(1 - position);
    }

    /**
     * @param position position of the extension of intake
     *                 Position: 0, fully upped(folded to the robot)
     *                 Position: 1, fully downed(extended)
     *                 position: 0.35 - IDLE
     *                 position: 0.73 - 5th highest cone(HIGH)
     *                 position: 0.75 - 4th highest cone
     *                 position: 0.79 - 3th highest cone
     *                 position: 0.85 - 2th highest cone
     *                 position: 0.87 - 1th highest cone(GROUND)
     */
    protected void setUpDownIntake(double position){
        sUpDownClaw1.setPosition(position);
        sUpDownClaw2.setPosition(position);
    }

    /**
     * @param position position of the rotation of the claw
     *                 Position: 0 - to grab the cone
     *                 position: 1 - to put on the basket
     */
    protected void setRotateClaw(double position){
        sRotateClaw.setPosition(position);
    }

    /**
     * @param position - position of the Claw
     *                 Position: 0 - hold the cone
     *                 Position: 0.5 - release the cone
     */
    protected void setClaw(double position){
        sClaw.setPosition(position);
    }

    /**
     * SETs Intake compartments to the IDLE position
     */
    private void IDLEIntakePosition(){
        setExtendIntake(1);
        setUpDownIntake(0.35);
        setRotateClaw(0);
        sleep(300);
    }

    /**
     * @param positionIntake - position of the Intake to be Set when at grabbing state
     * @param positionExtend - position of the Extend to be Set when at grabbing state
     */
    private void GRABIntakePosition(double positionIntake, double positionExtend){
            double overallTime = 1000 * (1 - positionExtend);
        setClaw(0.5);
        setExtendIntake(positionExtend);
        setUpDownIntake(0.5);
        sleep(100);
        setRotateClaw(0);
        sleep((long)(overallTime * 0.2));
        setUpDownIntake(positionIntake);
        sleep((long)(overallTime * 0.8));
        setClaw(0);
        sleep(300);
    }

    /**
     * Put cone to the Robot Basket
     * Standard position: UpDown Intake - 0.27
     *                    Extend Intake - 1
     */
    private void PUTCONEIntakePosition(){
        setUpDownIntake(0.6);
        sleep(100);
        setExtendIntake(1);
        setRotateClaw(1);
        sleep(700);
        setUpDownIntake(0.27);
        sleep(400);
        setClaw(0.5);
        sleep(500);
    }
}
