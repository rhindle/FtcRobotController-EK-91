package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name="elliot-and-jas", group="Proto")
//@Disabled
public class elliot1 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    Servo servoTest;

    @Override
    public void runOpMode() {

        double laptime=0;
        boolean Y_previous=false;
        servoTest =hardwareMap.get(Servo.class,"servo0");
        double servoAngle=0.5;

        //waitForStart();
        while (!isStarted()) {
            telemetry.addData("Status", "Press START");
            telemetry.update();
            sleep(100);
        }

        runtime.reset();

        while (opModeIsActive()) {
            servoTest.setPosition(servoAngle);

            if (gamepad1.right_bumper){
                servoAngle+=0.001;
                if (servoAngle>1) servoAngle=1;
            }
            if (gamepad1.left_bumper){
                servoAngle-=0.001;
                if (servoAngle<0) servoAngle=0;
            }

            if (gamepad1.y && !Y_previous){
                laptime=runtime.seconds();
                //Y_previous=true;
                runtime.reset();
            }
            Y_previous=gamepad1.y;

            if (gamepad1.b){
                servoAngle=.5;
//                servoTest.setPosition(.5);  does cool stuff B)
            }
            if (gamepad1.a){
                servoAngle=1;
//                servoTest.setPosition(1);
            }




            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("laptime",String.format("%.4f sec.",laptime));
            telemetry.update();
        }
    }
}
