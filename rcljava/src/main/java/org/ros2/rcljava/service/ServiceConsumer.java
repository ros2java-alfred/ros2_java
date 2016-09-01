package org.ros2.rcljava.service;

public interface ServiceConsumer<T, U> {

    public void call(T request, U response);

}
