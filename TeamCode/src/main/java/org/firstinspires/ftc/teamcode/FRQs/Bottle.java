package org.firstinspires.ftc.teamcode.FRQs;

public class Bottle
{
    private final double capacity;
    private double amountRemaining;

    public Bottle(double capacity)
    {
        this.capacity = capacity;
        amountRemaining = capacity;
    }

    public double updateAmount(double amountUsed)
    {
        amountRemaining -= amountUsed;

        if(amountRemaining < capacity * .25) {
            amountRemaining = capacity;
        }

        return amountRemaining;
    }
}