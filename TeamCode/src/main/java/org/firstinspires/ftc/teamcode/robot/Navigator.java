package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.utils.Functions;

public class Navigator {

   Robot robot;
   Telemetry telemetry;
   Drivetrain drivetrain;

   public double v0, v2, v1, v3;
   double driveSpeed, driveAngle, rotate;
   double maxSpeed = 1;
   public double storedHeading = 0;
   public double deltaHeading = 0;
   boolean useFieldCentricDrive = true;
   boolean useHeadingHold = true;
   boolean useHoldPosition = true;
   long headingDelay = System.currentTimeMillis();
   long idleDelay = System.currentTimeMillis();

//   public PIDCoefficients PIDmovement = new PIDCoefficients(0.06,0.0012,0.006); //.12 0 .035
//   public PIDCoefficients PIDrotate = new PIDCoefficients(0.026,0.01,0.00025);  // .03 0 0
//   PIDCoefficients PIDmovement_calculated = new PIDCoefficients(0,0,0);
//   PIDCoefficients PIDrotate_calculated = new PIDCoefficients(0,0,0);
//   long PIDTimeCurrent;
//   long PIDTimeLast;
//   double errorDistLast;
//   double errorRotLast;
//   double navAngleLast;

//   boolean onTargetByAccuracy = false;
//   public double targetX, targetY, targetRot;
//   int accurate = 1;  // 0 is loose, 1 is tight, more later?

//   double motorMinPower = 0.025;
//   double motorMaxPower = 1;

   /* Constructor */
   public Navigator(Robot robot){
      construct(robot);
   }

   void construct(Robot robot){
      this.robot = robot;
      this.telemetry = robot.telemetry;
//      this.localizer = robot.localizer;
      this.drivetrain = robot.drivetrain;
   }

   public void init() {
      v0 = 0.0;
      v1 = 0.0;
      v2 = 0.0;
      v3 = 0.0;

      storedHeading = 0;
      deltaHeading = 0;
   }

   public void loop() {

      if (!robot.useODO) {
         userDrive();
         drivetrain.setDrivePowers(v0, v1, v2, v3);
         return;
      }

//      userDrive();
//      if (idleDelay < System.currentTimeMillis() && useHoldPosition) {
//         autoDrive();
//      }
//      drivetrain.setDrivePowers(v0, v1, v2, v3);
   }

   // Determine motor speeds when under driver control
   public void userDrive () {

      driveSpeed = Math.pow(driveSpeed, 1);
      v0 = driveSpeed * (Math.cos(driveAngle / 180 * Math.PI) - Math.sin(driveAngle / 180 * Math.PI)) + rotate;
      v2 = driveSpeed * (Math.cos(driveAngle / 180 * Math.PI) + Math.sin(driveAngle / 180 * Math.PI)) + rotate;
      v1 = driveSpeed * (Math.cos(driveAngle / 180 * Math.PI) + Math.sin(driveAngle / 180 * Math.PI)) - rotate;
      v3 = driveSpeed * (Math.cos(driveAngle / 180 * Math.PI) - Math.sin(driveAngle / 180 * Math.PI)) - rotate;

      // scale so average motor speed is not more than maxSpeed
      // but only if maxspeed <> 1
      if (maxSpeed != 1) {
         double averageValue = JavaUtil.averageOfList(JavaUtil.createListWith(Math.abs(v0), Math.abs(v2), Math.abs(v1), Math.abs(v3)));
         averageValue = averageValue / maxSpeed;
         if (averageValue > 1) {
            v0 /= averageValue;
            v2 /= averageValue;
            v1 /= averageValue;
            v3 /= averageValue;
         }
      }

      // scale to no higher than 1
      double highValue = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(v0), Math.abs(v2), Math.abs(v1), Math.abs(v3), 1));
      v0 /= highValue;
      v2 /= highValue;
      v1 /= highValue;
      v3 /= highValue;
   }

   public void setUserDriveSettings(double driveSpeed, double driveAngle, double rotate) {
      if (!(driveSpeed == 0 && rotate == 0)) {
         idleDelay = System.currentTimeMillis() + 500;  //was 250 to match rotate?
      }
      this.driveSpeed = driveSpeed;
      this.driveAngle = driveAngle;
      this.rotate = rotate;
      // Modify for field centric Drive
      if (useFieldCentricDrive) {
         this.driveAngle = driveAngle - storedHeading + deltaHeading;
      }
      // Modify for Hold Angle
      if (useHeadingHold) {
         // Correct the heading if not currently being controlled
         // this should probably be incorporated into autodrive
         if (headingDelay <= System.currentTimeMillis()) {  // shouldn't need to check if == 0
            this.rotate = getError(storedHeading) / -15 * (driveSpeed + 0.2);   // base this on speed?
         }
      }
   }

//   // Determine motor speeds when under automatic control
//   public void autoDrive () {
//      double errorDist, deltaX, deltaY, errorRot, pDist, pRot, navAngle;
////      deltaX = targetX - robotPosition.X;  // error in x
////      deltaY = targetY - robotPosition.Y;  // error in y
//      // until we have a localizer, there is no deltaX or Y, but there can be a target rotation
//      deltaX = 0;
//      deltaY = 0;
////      telemetry.addData("DeltaX", JavaUtil.formatNumber(deltaX, 2));
////      telemetry.addData("DeltaY", JavaUtil.formatNumber(deltaY, 2));
//      errorRot = getError(targetRot);  // error in rotation   //20221222 added deltaheading!?
//      errorDist = Math.sqrt(Math.pow(deltaX,2) + Math.pow(deltaY,2));  // distance (error) from xy destination
//
//      //exit criteria if destination has been adequately reached
////      onTargetByAccuracy = false;
////      if (accurate==0 && errorDist<2) {  // no rotation component here
////         onTargetByAccuracy = true;
////      }
////      if (errorDist<0.5 && Math.abs(errorRot)<0.2) {
////         onTargetByAccuracy = true;
////      }
////      if (accurate==2 && errorDist<1 && Math.abs(errorRot)<1) {
////         onTargetByAccuracy = true;
////      }
////      if (accurate==3 && errorDist<2 && Math.abs(errorRot)<5) {  // ~ like 0 but still gets proportional
////         onTargetByAccuracy = true;
////      }
////      if (onTargetByAccuracy) {
////         navStep++;
////         return;
////      }
//
//      navAngle = Math.toDegrees(Math.atan2(deltaY,deltaX));  // angle to xy destination (vector when combined with distance)
//
//      PIDTimeCurrent = System.currentTimeMillis();
//
//      // Still need to reset I if we're going to use it.
//
//      if (Math.abs(navAngle-navAngleLast)>45) PIDmovement_calculated.i = 0;  // test - zero the I if we pass through an inflection
//
//      PIDmovement_calculated.p = PIDmovement.p * errorDist;
//      PIDmovement_calculated.i += PIDmovement.i * errorDist * ((PIDTimeCurrent - PIDTimeLast) / 1000.0);
//      PIDmovement_calculated.i = Math.max(Math.min(PIDmovement_calculated.i,1),-1);
//      PIDmovement_calculated.d = PIDmovement.d * (errorDist - errorDistLast) / ((PIDTimeCurrent - PIDTimeLast) / 1000.0);
//
//      pDist = PIDmovement_calculated.p + PIDmovement_calculated.i + PIDmovement_calculated.d;
//      pDist = Math.max(Math.min(pDist,motorMaxPower),motorMinPower);
//      if (accurate==0) pDist = 1;  // don't bother with proportional when hitting transitional destinations
//
//      PIDrotate_calculated.p = PIDrotate.p * errorRot;
//      PIDrotate_calculated.i += PIDrotate.i * errorRot * ((PIDTimeCurrent - PIDTimeLast) / 1000.0);
//      PIDrotate_calculated.i = Math.max(Math.min(PIDrotate_calculated.i,1),-1);
//      PIDrotate_calculated.d = PIDrotate.d * (errorDist - errorRotLast) / ((PIDTimeCurrent - PIDTimeLast) / 1000.0);
//
//      pRot =  PIDrotate_calculated.p + PIDrotate_calculated.i + PIDrotate_calculated.d;
//      pRot = Math.max(Math.min(Math.abs(pRot),motorMaxPower),motorMinPower)*Math.signum(pRot)*-1;
//
//      PIDTimeLast = PIDTimeCurrent;
//      errorDistLast = errorDist;
//      errorRotLast = errorRot;
//      navAngleLast = navAngle;
//
//      telemetry.addData("NavDistance", JavaUtil.formatNumber(errorDist, 2));
//      telemetry.addData("NavAngle", JavaUtil.formatNumber(navAngle, 2));
//      telemetry.addData("NavRotation", JavaUtil.formatNumber(errorRot, 2));
//      telemetry.addData("pDist", JavaUtil.formatNumber(pDist, 2));
//      telemetry.addData("pRot", JavaUtil.formatNumber(pRot, 2));
//
//      navAngle -= robot.imuHeading; //robotPosition.R;  // need to account for how the robot is oriented
//      double autoSpeed = pDist * 1;  // 1 here is maxspeed; could be turned into a variable
//      // the following adds the mecanum X, Y, and rotation motion components for each wheel
//      v0 = autoSpeed * (Math.cos(Math.toRadians(navAngle)) - Math.sin(Math.toRadians(navAngle))) + pRot;
//      v2 = autoSpeed * (Math.cos(Math.toRadians(navAngle)) + Math.sin(Math.toRadians(navAngle))) + pRot;
//      v1 = autoSpeed * (Math.cos(Math.toRadians(navAngle)) + Math.sin(Math.toRadians(navAngle))) - pRot;
//      v3 = autoSpeed * (Math.cos(Math.toRadians(navAngle)) - Math.sin(Math.toRadians(navAngle))) - pRot;
//
//      // scale to no higher than 1
//      double highValue = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(v0), Math.abs(v2), Math.abs(v1), Math.abs(v3), 1));
//      v0 /= highValue;
//      v2 /= highValue;
//      v1 /= highValue;
//      v3 /= highValue;
//   }

   public void handleRotate(double rotate) {
      // overall plan here is to deal with IMU latency
      if (rotate != 0) {
         storedHeading = robot.imuHeading; //robotPosition.R;
         headingDelay = System.currentTimeMillis() + 250;  // going to ignore the possibility of overflow
      } else if (headingDelay > System.currentTimeMillis()) {
         // keep re-reading until delay has passed
         storedHeading = robot.imuHeading; //robotPosition.R;
      }
   }

   public void setDeltaHeading() {
      deltaHeading = storedHeading;
   }

   public void toggleFieldCentricDrive() {
      useFieldCentricDrive = !useFieldCentricDrive;
   }

   public void toggleHeadingHold() {
      useHeadingHold = !useHeadingHold;
      storedHeading = robot.imuHeading; //robotPosition.R;
   }

   // Get heading error
   public double getError(double targetAngle) {
      double robotError;
      robotError = targetAngle - robot.imuHeading; //robotPosition.R;
      return Functions.normalizeAngle(robotError);
   }
}
