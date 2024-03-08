package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name="mec", group="Proto")
//@Disabled
public class mec extends LinearOpMode {

    DcMotorEx motorLF, motorRF, motorLB, motorRB;
    double Drive,Strafe,Turn;


    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        motorLB=hardwareMap.get(DcMotorEx.class, "motor2"); //0B
        motorRB=hardwareMap.get(DcMotorEx.class, "motor3"); //0B
        motorRF=hardwareMap.get(DcMotorEx.class, "motor1"); //0B
        motorLF=hardwareMap.get(DcMotorEx.class, "motor0"); //0B
        motorLF.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLB.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRB.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRF.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //waitForStart();
        while (!isStarted()) {
            telemetry.addData("Status", "Press START");
            telemetry.update();
            sleep(100);
        }

        runtime.reset();

        while (opModeIsActive()) {
            Drive=-gamepad1.left_stick_y;
            Strafe=gamepad1.left_stick_x;
            Turn=gamepad1.right_stick_x;
            motorLF.setPower(Drive+Strafe+Turn);
            motorLB.setPower(Drive-Strafe+Turn);
            motorRF.setPower(Drive-Strafe-Turn);
            motorRB.setPower(Drive+Strafe-Turn);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
