package frc.robot.core.utils;

public class Utils {
    public static String getSimpleClassName(String className) {
        int lastIndex = className.lastIndexOf('.');
        if (lastIndex != -1 && lastIndex < className.length() - 1) {
            return className.substring(lastIndex + 1);
        }
        return className;
    }

}