package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class TriSwerveDrive implements SwerveDrive, Runnable {

    private SwerveModule module1;
    private SwerveModule module2;
    private SwerveModule module3;
    BNO055IMU imu;

    double speed;
    double xSpeed;
    double ySpeed;
    double rotSpeed;

    DcMotor.RunMode mode;

    public TriSwerveDrive(
            DcMotor drive1, DcMotor swerve1, AnalogInput potentiometer1,
            DcMotor drive2, DcMotor swerve2, AnalogInput potentiometer2,
            DcMotor drive3, DcMotor swerve3, AnalogInput potentiometer3,
            BNO055IMU imu
            )
    {
        module1 = new SwerveModule(drive1, swerve1, potentiometer1, this);
        module2 = new SwerveModule(drive2, swerve2, potentiometer2, this);
        module3 = new SwerveModule(drive3, swerve3, potentiometer3, this);

        this.imu = imu;

        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    }

    @Override
    public void run()
    {

    }

    public void setRot(double rotSpeed)
    {
        this.rotSpeed = rotSpeed;
    }

    public void setSpeed(double speed)
    {
        this.speed = speed;
    }

    public void setDirection(double angle)
    {
        xSpeed = Math.cos(angle) * speed;
        ySpeed = Math.sin(angle) * speed;
    }

    public void setVelocity(double angle, double speed)
    {
        setSpeed(speed);
        setDirection(angle);
    }

    public void setMode(DcMotor.RunMode mode)
    {
        module1.setDriveMode(mode);
        module2.setDriveMode(mode);
        module3.setDriveMode(mode);

        this.mode = mode;
    }

    @Override
    public double getHeading()
    {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
    }
}
