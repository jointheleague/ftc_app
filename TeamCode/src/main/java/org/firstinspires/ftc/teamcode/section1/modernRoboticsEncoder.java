package org.firstinspires.ftc.teamcode.section1;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;

@I2cSensor(name = "Modern Robotics Motor Encoder", description = "Motor Encoder for Modern Robotics Motors", xmlTag = "MREncoder")
@Disabled
public class modernRoboticsEncoder extends I2cDeviceSynchDevice<I2cDeviceSynch>
{
    @Override
    public Manufacturer getManufacturer()
    {
        return Manufacturer.ModernRobotics;
    }

    @Override
    protected synchronized boolean doInitialize()
    {
        return true;
    }

    @Override
    public String getDeviceName()
    {

        return "Modern Robotics Motor Encoder";
    }

    public modernRoboticsEncoder(I2cDeviceSynch deviceClient)
    {
        super(deviceClient, true);

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }
}
