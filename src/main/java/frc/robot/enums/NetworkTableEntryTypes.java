package frc.robot.enums;

public enum NetworkTableEntryTypes {
    TARGET_X_AXIS("tx"),
    TARGET_Y_AXIS("ty"),
    TARGET_ID("tid"),
    TARGET_DETECTION("tv");

    public final String entryCode;

    private NetworkTableEntryTypes(String entryCode) {
        this.entryCode = entryCode;
    }
}
