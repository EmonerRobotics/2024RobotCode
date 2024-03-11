package frc.robot.modules.external.limelight;

public enum NetworkTableEntryType {
    TARGET_X_AXIS("tx"),
    TARGET_Y_AXIS("ty"),
    TARGET_ID("tid"),
    TARGET_DETECTION("tv");

    public final String entryCode;

    private NetworkTableEntryType(String entryCode) {
        this.entryCode = entryCode;
    }
}
