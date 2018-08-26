package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

public class SwerveDrive {

    private SwerveModule module1;
    private SwerveModule module2;
    private SwerveModule module3;

    public SwerveDrive(
            DcMotor drive1, DcMotor swerve1, AnalogInput potentiometer1,
            DcMotor drive2, DcMotor swerve2, AnalogInput potentiometer2,
            DcMotor drive3, DcMotor swerve3, AnalogInput potentiometer3
            )
    {
        module1 = new SwerveModule(drive1, swerve1, potentiometer1);
        module2 = new SwerveModule(drive2, swerve2, potentiometer2);
        module3 = new SwerveModule(drive3, swerve3, potentiometer3);
    }

    public void setAngle(int angle)
    {
        module1.rotateToDegree(angle);
        module2.rotateToDegree(angle);
        module3.rotateToDegree(angle);
    }
}
