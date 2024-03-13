package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name="launcher", group="Proto")
//@Disabled
public class launcher extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    Servo servoSwing;
    Servo servoTrigger;

    @Override
    public void runOpMode() {

        boolean Y_previous=false;
        boolean back_previous=false;
        servoSwing =hardwareMap.get(Servo.class,"servo0B");
        servoTrigger =hardwareMap.get(Servo.class,"servo1B");
        double servoAngle=0.59;

        //waitForStart();
        while (!isStarted()) {
            telemetry.addData("Status", "Press START");
            telemetry.update();
            sleep(100);
        }

        runtime.reset();

        while (opModeIsActive()) {
           // servoSwing.setPosition(servoAngle);


            if (gamepad1.right_bumper){
                servoAngle+=0.001;
                if (servoAngle>1) servoAngle=1;
            }
            if (gamepad1.left_bumper){
                servoAngle-=0.001;
                if (servoAngle<0.46) servoAngle=0.46;
            }

            if (gamepad1.dpad_up)
                servoSwing.setPosition(servoAngle);
            Y_previous=gamepad1.y;

            if (gamepad1.x){
                servoTrigger.setPosition(.702);
//                servoTest.setPosition(.5);  does cool stuff B)
            } else {
                servoTrigger.setPosition(.482);
            }

            if (gamepad1.back && !back_previous){
                // toggle whether the servo is enabled
                ServoControllerEx controller = (ServoControllerEx) servoSwing.getController();
                int servoPort = servoSwing.getPortNumber();
                if (controller.isServoPwmEnabled(servoPort)) controller.setServoPwmDisable(servoPort);
                else controller.setServoPwmEnable(servoPort);
            }
            back_previous=gamepad1.back;


            telemetry.addData("SwingPosition", servoAngle);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
          //  telemetry.addData("laptime",String.format("%.4f sec.",laptime));
            telemetry.update();
        }
    }
}
