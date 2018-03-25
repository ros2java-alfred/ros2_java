/* Copyright 2016-2018 Mickael Gaillard <mick.gaillard@gmail.com>
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

package org.ros2.rcljava.node.parameter;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;

import org.ros2.rcljava.node.Node;
import org.ros2.rcljava.node.service.Client;
import org.ros2.rcljava.node.topic.Subscription;
import org.ros2.rcljava.node.topic.SubscriptionCallback;
import org.ros2.rcljava.node.topic.Topics;
import org.ros2.rcljava.qos.QoSProfile;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import rcl_interfaces.msg.ListParametersResult;
import rcl_interfaces.msg.Parameter;
import rcl_interfaces.msg.ParameterDescriptor;
import rcl_interfaces.msg.ParameterEvent;
import rcl_interfaces.msg.ParameterValue;
import rcl_interfaces.msg.SetParametersResult;

import rcl_interfaces.srv.DescribeParameters;
import rcl_interfaces.srv.DescribeParameters_Request;
import rcl_interfaces.srv.DescribeParameters_Response;
import rcl_interfaces.srv.GetParameterTypes;
import rcl_interfaces.srv.GetParameterTypes_Request;
import rcl_interfaces.srv.GetParameterTypes_Response;
import rcl_interfaces.srv.GetParameters;
import rcl_interfaces.srv.GetParameters_Request;
import rcl_interfaces.srv.GetParameters_Response;
import rcl_interfaces.srv.ListParameters;
import rcl_interfaces.srv.ListParameters_Request;
import rcl_interfaces.srv.ListParameters_Response;
import rcl_interfaces.srv.SetParameters;
import rcl_interfaces.srv.SetParameters_Request;
import rcl_interfaces.srv.SetParameters_Response;

/**
 * Sync Parameter Client.
 *
 */
public class SyncParametersClient {

    private static final Logger logger = LoggerFactory.getLogger(SyncParametersClient.class);

    private final Node ownerNode;
    private final String remoteNodeName;
    private Client<GetParameters> getParametersClient;
    private Client<GetParameterTypes> getParameterTypesClient;
    private Client<SetParameters> setParametersClient;
    private Client<ListParameters> listParametersClient;
    private Client<DescribeParameters> describeParametersClient;

    public SyncParametersClient(final Node node) {
        this(node, null, QoSProfile.PARAMETER);
    }

    public SyncParametersClient(final Node node, final String remoteNodeName) {
        this(node, remoteNodeName, QoSProfile.PARAMETER);
    }

    public SyncParametersClient(final Node node, final QoSProfile profileParameter) {
        this(node, null, profileParameter);
    }

    public SyncParametersClient(final Node node, final String remoteNodeName, final QoSProfile profileParameter) {
        this.ownerNode = node;
        if (remoteNodeName == null || remoteNodeName.equals("")) {
            this.remoteNodeName = node.getName();
        } else {
            this.remoteNodeName = remoteNodeName;
        }

        logger.debug(String.format("Create parameter client for %s", this.remoteNodeName));

        try {
            this.getParametersClient = this.ownerNode.<GetParameters>createClient(
                    GetParameters.class,
                    String.format(ParameterService.TOPIC_GETPARAMETERS, this.remoteNodeName),
                    profileParameter);
            this.getParameterTypesClient = this.ownerNode.<GetParameterTypes>createClient(
                    GetParameterTypes.class,
                    String.format(ParameterService.TOPIC_GETPARAMETERTYPES, this.remoteNodeName),
                    profileParameter);
            this.setParametersClient = this.ownerNode.<SetParameters>createClient(
                    SetParameters.class,
                    String.format(ParameterService.TOPIC_SETPARAMETERS, this.remoteNodeName),
                    profileParameter);
            this.listParametersClient = this.ownerNode.<ListParameters>createClient(
                    ListParameters.class,
                    String.format(ParameterService.TOPIC_LISTPARAMETERS, this.remoteNodeName),
                    profileParameter);
            this.describeParametersClient = this.ownerNode.<DescribeParameters>createClient(
                    DescribeParameters.class,
                    String.format(ParameterService.TOPIC_DESCRIBEPARAMETERS, this.remoteNodeName),
                    profileParameter);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * Safely destroy the underlying ROS2 subscriber structure.
     */
    public void dispose() {
        logger.debug("Destroy parameter client : " + this.remoteNodeName);

        this.getParametersClient.dispose();
        this.getParameterTypesClient.dispose();
        this.setParametersClient.dispose();
        this.listParametersClient.dispose();
        this.describeParametersClient.dispose();
    }

    public List<ParameterVariant<?>> getParameters(final List<String> list) {
        List<ParameterVariant<?>> result = null;

        final GetParameters_Request request = new GetParameters_Request();
        request.setNames(list);

        // Call service...
        final Future<GetParameters_Response> future = this.getParametersClient.sendRequest(request);

        if (future != null) {
            try {
                logger.debug("Call get Parameter service.");
                final List<ParameterValue> values = future.get().getValues();

                result = new ArrayList<ParameterVariant<?>>();
                for (int i = 0; i < values.size(); i++) {
                    final ParameterValue parameterValue = values.get(i);

                    final Parameter parameter = new Parameter();
                    parameter.setName(request.getNames().get(i));
                    parameter.setValue(parameterValue);

                    final ParameterVariant<?> parameterVariant = ParameterVariant.fromParameter(parameter);
                    result.add(parameterVariant);
                }

            } catch (InterruptedException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            } catch (ExecutionException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        } else {
            logger.error("getParameters was interrupted. Exiting.");
        }

        return result;
    }

    public List<Byte> getParameterTypes(final List<String> parameterNames) {
        List<Byte> result = null;

        final GetParameterTypes_Request request = new GetParameterTypes_Request();
        request.setNames(parameterNames);

        // Call service...
        final Future<GetParameterTypes_Response> future = this.getParameterTypesClient.sendRequest(request);

        if (future != null) {
            try {
                logger.debug("Call get Type of Parameter service.");
                result =  future.get().getTypes();
            } catch (InterruptedException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            } catch (ExecutionException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        } else {
            logger.error("getParameterTypes was interrupted. Exiting.");
        }

        return result;
    }

    public boolean hasParameter(final String parameterName) {
        final List<String> finded = Arrays.asList(parameterName);
        logger.debug("Has Parameter service.");
        final ListParametersResult result = this.listParameters(finded, 1);
        return result.getNames().size() > 0;
    }

    public ListParametersResult listParameters(final List<String> prefixes, final int depth) {
        ListParametersResult result = null;

        // Set request.
        final ListParameters_Request request = new ListParameters_Request();
        request.setPrefixes(prefixes);
        request.setDepth(depth);

        // Call service...
        final Future<ListParameters_Response> future = this.listParametersClient.sendRequest(request);

        if (future != null) {
            try {
                logger.debug("Call list of Parameter service.");
                result = future.get().getResult();
            } catch (InterruptedException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            } catch (ExecutionException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        } else {
            logger.error("listParameters was interrupted. Exiting.");
        }

        return result;
    }

    public List<SetParametersResult> setParameters(final List<ParameterVariant<?>> list) {
        List<SetParametersResult> result = null;

        // Set request.
        final SetParameters_Request request = new SetParameters_Request();

        final List<Parameter> convList = new ArrayList<Parameter>();
        for (final ParameterVariant<?> parameterVariant : list) {
            final Parameter param = parameterVariant.toParameter();
            convList.add(param);
        }
        request.setParameters(convList);

        // Call service...
        final Future<SetParameters_Response> future = this.setParametersClient.sendRequest(request);

        if (future != null) {
            try {
                logger.debug("Call set Parameter service.");
                result = future.get().getResults();
            } catch (InterruptedException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            } catch (ExecutionException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        } else {
            logger.error("setParameters was interrupted. Exiting.");
        }

        return result;
    }

    public List<ParameterDescriptor> describeParametersClient(final List<String> parameterNames) {
        List<ParameterDescriptor> result = null;

        final DescribeParameters_Request request = new DescribeParameters_Request();
        request.setNames(parameterNames);

        // Call service...
        final Future<DescribeParameters_Response> future = this.describeParametersClient.sendRequest(request);

        if (future != null) {
            try {
                logger.debug("Call describe Parameter service.");
                result = future.get().getDescriptors();
            } catch (InterruptedException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            } catch (ExecutionException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        } else {
            logger.error("setParameters was interrupted. Exiting.");
        }

        return result;
    }

    public Subscription<ParameterEvent> onParameterEvent(final ParameterEventCallback parameterConsumer) {
        final Subscription<ParameterEvent> sub_event = this.ownerNode.<ParameterEvent>createSubscription(
                ParameterEvent.class,
                Topics.PARAM_EVENT,
                new SubscriptionCallback<ParameterEvent>() {
                    @Override
                    public void dispatch(final ParameterEvent msg) {
                        parameterConsumer.onEvent(msg);
                    }
                },
                QoSProfile.PARAMETER_EVENTS);

        return sub_event;
    }

}
