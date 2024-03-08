package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

@TeleOp (name="elliot2", group="Proto")
//@Disabled
public class elliot2 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    Servo servoTest;
    DcMotorEx tapeMotor;  //, testMotor;
    float encPerInch = 31.4F;
    boolean manualControl = true;
    int tapeEncoder = 0;
    final int maxPos = (int)(24 * encPerInch);

    @Override
    public void runOpMode() {

        boolean Y_previous=false;
        boolean X_previous=false;
        int tapePos = 0;  // moved
        servoTest=hardwareMap.get(Servo.class,"servo0");
        tapeMotor=hardwareMap.get(DcMotorEx.class, "motor3B"); //0B
        tapeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        tapeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        tapeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        tapeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        overrideMotorSettings(tapeMotor);

        //waitForStart();
        while (!isStarted()) {
            telemetry.addData("Status", "Press START");
            telemetry.update();
            sleep(100);
        }

        runtime.reset();

        while (opModeIsActive()) {

            tapeEncoder = tapeMotor.getCurrentPosition();

            double motorPower = gamepad1.right_trigger - gamepad1.left_trigger;

            if (motorPower!=0) {
                if (tapeMotor.getMode()!= DcMotorEx.RunMode.RUN_USING_ENCODER) {
                    tapeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                }
                if (tapeEncoder<=0 && motorPower<0 && !gamepad1.left_bumper) motorPower=0;
                if (tapeEncoder>=maxPos && motorPower>0) motorPower=0;
                tapeMotor.setPower(motorPower);
                manualControl = true;
            }
            if (motorPower==0 && manualControl) tapeMotor.setPower(0);
            if (gamepad1.b) {
                tapeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                tapeEncoder = 0;
            }
            if (gamepad1.y && !Y_previous){
                manualControl = false;  //oops
                tapeMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                tapePos = tapeEncoder;
                tapePos+=(int)encPerInch;
                tapePos = Math.min(tapePos, maxPos);
                tapeMotor.setTargetPosition(tapePos);
                tapeMotor.setPower(1);
            }
            Y_previous=gamepad1.y;

            if (gamepad1.x && !X_previous){
                if (manualControl) {
                    manualControl = false;
                    tapeMotor.setTargetPosition(tapeEncoder);
                    tapeMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                }
                tapePos = tapeMotor.getTargetPosition()+2;  // the +2 is for rounding and position tolerance
                int tapePosInch = (int)(tapePos/encPerInch);
                tapePos = (int)(encPerInch*(tapePosInch+1));
                tapePos = Math.min(tapePos, maxPos);
                tapeMotor.setTargetPosition(tapePos);
                tapeMotor.setPower(1);
            }
            X_previous=gamepad1.x;

            telemetry.addData("tapePos", tapePos);
            telemetry.addData("motoPos", tapeEncoder);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }

    public void overrideMotorSettings(DcMotorEx mtr) {
        // This will assure the motor is configured like a RevRoboticsCoreHexMotor regardless of the RC config
        MotorConfigurationType motorCfg = mtr.getMotorType().clone();
        // put a breakpoint in here to see all the motor settings.  See also:
        // https://github.com/OpenFTC/Extracted-RC/tree/master/Hardware/src/main/java/com/qualcomm/hardware/motors
        motorCfg.setAchieveableMaxRPMFraction(0.85);
        motorCfg.setGearing(36.25);
        motorCfg.setMaxRPM(137);
        motorCfg.setOrientation(Rotation.CCW);
        motorCfg.setTicksPerRev(288);
        mtr.setMotorType(motorCfg);

        mtr.setPIDCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDCoefficients(10,3,0));
        mtr.setPIDCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, new PIDCoefficients(10,0.05,0));
    }
}

//        mtr.setVelocityPIDFCoefficients(10,3,0,0);
//        mtr.setPositionPIDFCoefficients(20);

        /* GoBilda
        motorCfg.setAchieveableMaxRPMFraction(0.85);
        motorCfg.setGearing(99.5);
        motorCfg.setMaxRPM(60.0);
        motorCfg.setOrientation(Rotation.CCW);
        motorCfg.setTicksPerRev(2786.0);
        */

// https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/322
// https://github.com/OpenFTC/Extracted-RC/blob/master/Hardware/src/main/java/com/qualcomm/hardware/motors/RevRoboticsCoreHexMotor.java
//@MotorType(ticksPerRev=288, gearing=36.25, maxRPM=137, orientation= Rotation.CCW)
//@ExpansionHubMotorControllerVelocityParams(P=10, I=3, D=0)
//@ExpansionHubMotorControllerPositionParams(P=10, I=0.05, D=0)
//@DeviceProperties(xmlTag="RevRoboticsCoreHexMotor", name="@string/rev_core_hex_name", builtIn = true)
//@DistributorInfo(distributor="@string/rev_distributor", model="REV-41-1300", url="http://www.revrobotics.com/rev-41-1300")
//https://ftc9929.com/2019/12/16/stress-free-ftc-hardware-configurations/