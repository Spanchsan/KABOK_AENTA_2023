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
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Main Test")
public class MainTest extends LinearOpMode {

    Servo sExtend1, sExtend2, sUpDownClaw1, sUpDownClaw2, sRotateClaw, sClaw;
    DcMotor m0, m1;

    @Override
    public void runOpMode() {
        sExtend1 = hardwareMap.servo.get("serv0");
        sExtend2 = hardwareMap.servo.get("serv1");
        sUpDownClaw1 = hardwareMap.servo.get("serv2");
        sUpDownClaw2 = hardwareMap.servo.get("serv3");
        sRotateClaw = hardwareMap.servo.get("serv4");
        sClaw = hardwareMap.servo.get("serv5");
        m0 = hardwareMap.dcMotor.get("motor0");
        m1 = hardwareMap.dcMotor.get("motor1");
        sExtend1.scaleRange(0, 0.92);
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.right_bumper) {
                sExtend1.setPosition(sExtend1.getPosition() + 0.003);
                sExtend2.setPosition(sExtend2.getPosition() - 0.003);
            } else if(gamepad1.left_bumper) {
                sExtend1.setPosition(sExtend1.getPosition() - 0.003);
                sExtend2.setPosition(sExtend2.getPosition() + 0.003);
            }
            if(gamepad1.dpad_up){
                sUpDownClaw1.setPosition(sUpDownClaw1.getPosition() + 0.003);
                sUpDownClaw2.setPosition(sUpDownClaw2.getPosition() + 0.003);
            } else if(gamepad1.dpad_down){
                sUpDownClaw1.setPosition(sUpDownClaw1.getPosition() - 0.003);
                sUpDownClaw2.setPosition(sUpDownClaw2.getPosition() - 0.003);
            }
            if(gamepad1.b){
                sRotateClaw.setPosition(0);
            } else if(gamepad1.x){
                sRotateClaw.setPosition(1);
            }
            if(gamepad1.a){
                sClaw.setPosition(0);
            } else if(gamepad1.y){
                sClaw.setPosition(0.5);
            }
            if(gamepad1.dpad_left){
                sExtend1.setPosition(1);
                sExtend2.setPosition(0);
            } else if(gamepad1.dpad_right){
                sExtend1.setPosition(0);
                sExtend2.setPosition(1);
            }
            if(gamepad1.right_stick_button){
                IDLEIntakePosition();
                GRABIntakePosition(0.79, 0);
                PUTCONEIntakePosition();
                GRABIntakePosition(0.83, 0);
                PUTCONEIntakePosition();
                GRABIntakePosition(0.87, 0);
                PUTCONEIntakePosition();
                setUpDownIntake(0.32);
                sleep(200);
                setRotateClaw(0);
            }
            m0.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            m1.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
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
        setClaw(0.5);
        setExtendIntake(positionExtend);
        setUpDownIntake(0.5);
        sleep(100);
        setRotateClaw(0);
        sleep(200);
        setUpDownIntake(positionIntake);
        sleep(800);
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
