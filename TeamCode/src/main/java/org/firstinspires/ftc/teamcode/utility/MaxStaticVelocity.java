package org.firstinspires.ftc.teamcode.utility;

public class MaxStaticVelocity {
    // Max Velocity of Flywheel Storage
    public static double maxStaticVelocity = 0.0;

    // True max velocity for PIDF
    /*
     * See
     * https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
     * for more details
    */
    public static double maxStaticPIDFVelocity = 32767 / maxStaticVelocity;
}
