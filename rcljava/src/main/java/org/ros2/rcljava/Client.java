/* Copyright 2016 Esteve Fernandez <esteve@apache.org>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package org.ros2.rcljava;

import java.lang.ref.WeakReference;
import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.Future;

public class Client<T> {
  static {
    try {
      System.loadLibrary("rcljavaClient__" + RCLJava.getRMWIdentifier());
    } catch (UnsatisfiedLinkError ule) {
      System.err.println("Native code library failed to load.\n" + ule);
      System.exit(1);
    }
  }

  private final WeakReference<Node> nodeReference;
  private final long nodeHandle;
  private final long clientHandle;
  private final Class<T> serviceType;
  private final String serviceName;
  private long sequenceNumber = 0;
  private Map<Long, RCLFuture> pendingRequests;

  private long requestFromJavaConverterHandle = 0;
  private long requestToJavaConverterHandle = 0;

  private long responseFromJavaConverterHandle = 0;
  private long responseToJavaConverterHandle = 0;

  private final Class requestType;
  private final Class responseType;

  public Client(final WeakReference<Node> nodeReference,
      final long nodeHandle, final long clientHandle, final Class<T> serviceType, final String serviceName,
      final Class requestType, final Class responseType,
      final long requestFromJavaConverterHandle, final long requestToJavaConverterHandle,
      final long responseFromJavaConverterHandle, final long responseToJavaConverterHandle) {
    this.nodeReference = nodeReference;
    this.nodeHandle = nodeHandle;
    this.clientHandle = clientHandle;
    this.serviceType = serviceType;
    this.serviceName = serviceName;
    this.requestType = requestType;
    this.responseType = responseType;
    this.requestFromJavaConverterHandle = requestFromJavaConverterHandle;
    this.requestToJavaConverterHandle = requestToJavaConverterHandle;
    this.responseFromJavaConverterHandle = responseFromJavaConverterHandle;
    this.responseToJavaConverterHandle = responseToJavaConverterHandle;
    this.pendingRequests = new HashMap<Long, RCLFuture>();
  }

  public <U, V> Future<V> sendRequest(U request) {
    synchronized(pendingRequests) {
      sequenceNumber++;
      nativeSendClientRequest(clientHandle, sequenceNumber,
      requestFromJavaConverterHandle, requestToJavaConverterHandle, request);
      RCLFuture<V> future = new RCLFuture<V>(this.nodeReference);
      pendingRequests.put(sequenceNumber, future);
      return future;
    }
  }

  public <U> void handleResponse(RMWRequestId header, U response) {
    synchronized(pendingRequests) {
      long sequenceNumber = header.sequenceNumber;
      RCLFuture<U> future = pendingRequests.remove(sequenceNumber);
      future.set(response);
    }
  }

  public final Class<T> getServiceType() {
    return serviceType;
  }

  public final long getClientHandle() {
    return clientHandle;
  }

  private static native void nativeSendClientRequest(long clientHandle, long sequenceNumber,
      long requestFromJavaConverterHandle, long requestToJavaConverterHandle, Object requestMessage);


  public final long getRequestFromJavaConverterHandle() {
    return this.requestFromJavaConverterHandle;
  }

  public final long getRequestToJavaConverterHandle() {
    return this.requestToJavaConverterHandle;
  }

  public final long getResponseFromJavaConverterHandle() {
    return this.responseFromJavaConverterHandle;
  }

  public final long getResponseToJavaConverterHandle() {
    return this.responseToJavaConverterHandle;
  }

  public final Class getRequestType() {
    return this.requestType;
  }

  public final Class getResponseType() {
    return this.responseType;
  }
}
