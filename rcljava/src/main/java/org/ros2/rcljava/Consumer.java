package org.ros2.rcljava;

public interface Consumer<T> {
    void accept(T t);
}
