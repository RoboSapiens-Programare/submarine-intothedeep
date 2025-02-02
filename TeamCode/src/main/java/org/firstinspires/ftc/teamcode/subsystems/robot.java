package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;


public class robot {
    private boolean initialize;

    public robot(HardwareMap hardwareMap){
        initialize = false;
    }

    public boolean isInitialize() {return initialize;}
}
