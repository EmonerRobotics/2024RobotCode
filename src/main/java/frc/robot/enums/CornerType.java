package frc.robot.enums;

public enum CornerType {
    FrontLeft(0),
    FrontRight(1),
    RearLeft(2),
    RearRight(3);

    public final int index;

    CornerType(int index) {
        this.index = index;
    }
}