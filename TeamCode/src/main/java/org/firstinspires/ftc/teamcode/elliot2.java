package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.DistributorInfo;
import com.qualcomm.robotcore.hardware.configuration.ExpansionHubMotorControllerPositionParams;
import com.qualcomm.robotcore.hardware.configuration.ExpansionHubMotorControllerVelocityParams;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

@TeleOp (name="elliot2", group="Proto")
//@Disabled
public class elliot2 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    Servo servoTest;
    DcMotor tapeMotor;
    float encPerInch = 31.4F;
    boolean manualControl = true;

    @Override
    public void runOpMode() {

        boolean Y_previous=false;
        int tapePos = 0;  // moved
        servoTest =hardwareMap.get(Servo.class,"servo0");
        tapeMotor =hardwareMap.get(DcMotor.class, "motor3B"); //0B
        tapeMotor.setDirection(DcMotorSimple.Direction.REVERSE);  // FORWARD
        tapeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tapeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tapeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //10 inches = 314 encoder clicks
       // tapeMotor.setMotorType(@MotorType(ticksPerRev = 288, gearing = 36.25, maxRPM = 137, orientation = Rotation.CCW));
        //tapeMotor.setMotorType(new MotorConfigurationType().setGearing(36.25));
       // tapeMotor.setMotorType(RevRoboticsCoreHexMotor);
        //new MotorConfigurationType().setTicksPerRev(288)
        //288, 36.25, 137, Rotation.CCW
       // ticksPerRev=288, gearing=36.25, maxRPM=137, orientation=Rotation.CCW)

        //waitForStart();
        while (!isStarted()) {
            telemetry.addData("Status", "Press START");
            telemetry.update();
            sleep(100);
        }

        runtime.reset();

        while (opModeIsActive()) {

            double motorPower = gamepad1.right_trigger - gamepad1.left_trigger;

            if (motorPower!=0) {
                if (tapeMotor.getMode()!= DcMotor.RunMode.RUN_USING_ENCODER) {
                    tapeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                tapeMotor.setPower(motorPower);
                manualControl = true;
            }
            if (motorPower==0 && manualControl) tapeMotor.setPower(0);

            if (gamepad1.y && !Y_previous){
                manualControl = false;  //oops
                tapeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                tapePos = tapeMotor.getCurrentPosition();
                tapePos+=(int)encPerInch;
                tapeMotor.setTargetPosition(tapePos);
                tapeMotor.setPower(1);
            }
            Y_previous=gamepad1.y;

            telemetry.addData("tapePos", tapePos);
            telemetry.addData("motoPos", tapeMotor.getCurrentPosition());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}

// https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/322
// https://github.com/OpenFTC/Extracted-RC/blob/master/Hardware/src/main/java/com/qualcomm/hardware/motors/RevRoboticsCoreHexMotor.java
//@MotorType(ticksPerRev=288, gearing=36.25, maxRPM=137, orientation= Rotation.CCW)
//@ExpansionHubMotorControllerVelocityParams(P=10, I=3, D=0)
//@ExpansionHubMotorControllerPositionParams(P=10, I=0.05, D=0)
//@DeviceProperties(xmlTag="RevRoboticsCoreHexMotor", name="@string/rev_core_hex_name", builtIn = true)
//@DistributorInfo(distributor="@string/rev_distributor", model="REV-41-1300", url="http://www.revrobotics.com/rev-41-1300")

//https://ftc9929.com/2019/12/16/stress-free-ftc-hardware-configurations/