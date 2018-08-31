
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
        module.rotateToDegree((gamepad1.left_stick_x != 0) ? Math.atan(-gamepad1.left_stick_y / gamepad1.left_stick_x) : ((gamepad1.left_stick_y < 0) ? 90 : -90));
        module.setDrivePower(-gamepad1.right_stick_y);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
