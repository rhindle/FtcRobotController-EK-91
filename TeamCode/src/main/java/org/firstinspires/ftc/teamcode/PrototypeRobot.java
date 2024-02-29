package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Controls;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp (name="PrototypeRobot", group="Proto")
//@Disabled
public class PrototypeRobot extends LinearOpMode {

   Robot robot;
   public Controls controls;
//   ButtonMgr buttonMgr;
   ElapsedTime loopElapsedTime = new ElapsedTime();

   @Override
   public void runOpMode() {
      robot = new Robot(this);
      robot.init();

      controls = new Controls(robot);

      // Send telemetry message to alert driver that we are calibrating;
      telemetry.addData(">", "Calibrating Gyro");    //
      telemetry.update();
      // make sure the gyro is calibrated before continuing
      while (!isStopRequested() && !robot.sensorIMU.isGyroCalibrated())  {
         sleep(50);
         idle();
      }

      telemetry.addData(">", "Robot Ready.");    //
      telemetry.update();

      // Wait for the game to start (Display Gyro value), and reset gyro before we move..
      while (!isStarted()) {
         robot.buttonMgr.loop();
         telemetry.addData(">", "Robot Heading = %.1f", robot.returnImuHeading(true));
         telemetry.update();
         sleep(100);
      }

      robot.drivetrain.init();
      robot.navigator.init();

      // run until the end of the match (driver presses STOP)
      while (opModeIsActive()) {

         robot.loop();  // this will take care of clearing out the bulk reads
         robot.buttonMgr.loop();

         controls.loop();
         robot.navigator.loop();

         telemetry.addData("Heading", "%.1f", robot.returnImuHeading());
         telemetry.addData("LoopTime(ms)","%.1f",loopElapsedTime.milliseconds());
         telemetry.addData("LoopSpeed(lps)","%.1f",1/(loopElapsedTime.milliseconds()/1000));
         loopElapsedTime.reset();
         telemetry.update();
      }
   }
}