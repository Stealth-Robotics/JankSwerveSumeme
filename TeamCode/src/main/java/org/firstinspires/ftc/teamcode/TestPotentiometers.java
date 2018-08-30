
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.AnalogInput;


@TeleOp(name="TestPotentiometer", group="Iterative Opmode")
public class TestPotentiometers extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private AnalogInput potentiometer1;
    private AnalogInput potentiometer2;
    private AnalogInput potentiometer3;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        potentiometer1 = hardwareMap.get(AnalogInput.class, "potentiometer1");
        potentiometer2 = hardwareMap.get(AnalogInput.class, "potentiometer2");
        potentiometer3 = hardwareMap.get(AnalogInput.class, "potentiometer3");
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
    public void loop() {
        telemetry.addData("Potertiometer1: ", potentiometer1.getVoltage());
        telemetry.addData("Potertiometer2: ", potentiometer2.getVoltage());
        telemetry.addData("Potertiometer3: ", potentiometer3.getVoltage());
        telemetry.addData("Max Voltages: ", "" + potentiometer1.getMaxVoltage() + " " + potentiometer2.getMaxVoltage() + " " + potentiometer3.getMaxVoltage());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
