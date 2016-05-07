package org.ros2.rosidl_generator_java;

import java.lang.reflect.Method;

public class RCLJavaProxy {
    public static synchronized String getTypesupportIdentifier() {
        try {
            Class c = Class.forName("org.ros2.rcljava.RCLJava");
            Method m = c.getDeclaredMethod("getTypesupportIdentifier", (Class<?> []) null);
            Object o = m.invoke(null, (Class<?> []) null);
            return (String)o;
        } catch(Exception e) {
            // TODO(esteve): handle exception
            return null;
        }
    }

    public static synchronized String getRMWIdentifier() {
        try {
            Class c = Class.forName("org.ros2.rcljava.RCLJava");
            Method m = c.getDeclaredMethod("getRMWIdentifier", (Class<?> []) null);
            Object o = m.invoke(null, (Class<?> []) null);
            return (String)o;
        } catch(Exception e) {
            // TODO(esteve): handle exception
            return null;
        }
    }

}
