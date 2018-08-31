
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="TestSwerveModule", group="Iterative Opmode")
public class TestSwerveModule extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private SwerveModule module;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        module = new SwerveModule((DcMotor) hardwareMap.get("drive1"), (DcMotor) hardwareMap.get("swerve1"),
                hardwareMap.get(AnalogInput.class, "potentiometer1"), 0, 0, 0);
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
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
    public void loop()
    {
        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        //the length of the joystick vector
        double pow = Math.sqrt(x * x + y * y);
        //joystick deadzone
        if(pow > 0.1) {
            //the angle (forward is 0 degrees, and it still has range -90 to 90)
            //turns out atan2(x,y) does atan(x/y) but without the division by zero problem
            //and returns the correct value
            //I use x/y intentionally because i want 0 degrees along the y axis.
            //I don't negate y because i want positive angles to be counterclockwise per right-hand rule
            //(the imu angles are like this as well)

            //TLDR this should all work right and is reusable in crab mode. Also be checking these angles,
            //ensure they are as expected and that rotation of the swerve module has right-hand-ruley angles
            double angle = Math.atan2(x, y);
            angle *= 180.0 / Math.PI; //convert to degrees
            if(y < 0){
                //if we have a negative y we need to flip our angle
                angle += 180;
            }
            module.rotateToDegree(angle);
            module.setDrivePower(pow);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
