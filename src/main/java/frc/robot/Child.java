package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Child extends SubsystemBase {
    private int number = 0;

    public Child() {
        Shuffleboard.getTab("Joshua V's many children")
                .addNumber("Number of Joshua V's Children", this::getChildren);
    }

    public void addOne() {
        number++;
    }

    public void times2() {
        number *= 2;
    }

    public Command add() {
        return Commands.runOnce(this::addOne, this);
    }

    public Command doubleChildren() {
        return Commands.runOnce(this::times2, this);
    }

    public int getChildren() {
        return number;
    }

    @Override
    public String toString() {
        return "You have fathered " + number + " children!";
    }
}
