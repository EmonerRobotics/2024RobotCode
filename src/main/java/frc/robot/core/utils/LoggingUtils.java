package frc.robot.core.utils;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import static frc.robot.core.utils.Utils.getSimpleClassName;

public class LoggingUtils {
    private static final Logger logger = LoggerFactory.getLogger(LoggingUtils.class);


    public static void logMessage(String message) {
        StackTraceElement[] stackTraceElements = Thread.currentThread().getStackTrace();
        if (stackTraceElements.length > 2) {
            String callerClassName = getSimpleClassName(stackTraceElements[2].getClassName());
            String callerMethodName = stackTraceElements[2].getMethodName();
            logger.warn("[FRC-7840][" + callerClassName + "][" + callerMethodName + "] " + message);
        } else {
            logger.warn("[FRC-7840] " + message);
        }
    }

    public static void logEvent() {
        StackTraceElement[] stackTraceElements = Thread.currentThread().getStackTrace();
        if (stackTraceElements.length > 2) {
            String callerClassName = getSimpleClassName(stackTraceElements[2].getClassName());
            String callerMethodName = stackTraceElements[2].getMethodName();
            logger.warn("[FRC-7840][" + callerClassName + "][" + callerMethodName + "] ");
        } else {
            logger.warn("[FRC-7840] cannot find caller class or method");
        }
    }

}