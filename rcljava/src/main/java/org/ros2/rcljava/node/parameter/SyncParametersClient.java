/* Copyright 2016 Open Source Robotics Foundation, Inc.
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
import java.util.List;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;
import java.util.logging.Logger;

import org.ros2.rcljava.qos.QoSProfile;
import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.node.Node;
import org.ros2.rcljava.node.service.Client;
import org.ros2.rcljava.node.topic.Subscription;

import rcl_interfaces.srv.GetParameters;
import rcl_interfaces.srv.GetParameters_Request;
import rcl_interfaces.srv.GetParameters_Response;
import rcl_interfaces.srv.GetParameterTypes;
import rcl_interfaces.srv.SetParameters;
import rcl_interfaces.srv.SetParameters_Request;
import rcl_interfaces.srv.SetParameters_Response;
import rcl_interfaces.srv.ListParameters;
import rcl_interfaces.srv.DescribeParameters;
import rcl_interfaces.msg.ListParametersResult;
import rcl_interfaces.msg.Parameter;
import rcl_interfaces.msg.ParameterType;
import rcl_interfaces.msg.ParameterValue;
import rcl_interfaces.msg.SetParametersResult;

/**
 * Parameter Client.
 *
 * @param <T> Service Type.
 * @author Mickael Gaillard <mick.gaillard@gmail.com>
 */
public class SyncParametersClient {

    private static Logger logger = Logger.getLogger(RCLJava.LOG_NAME);

    private Client<GetParameters> getParametersClient;
    private Client<GetParameterTypes> getParameterTypesClient;
    private Client<SetParameters> setParametersClient;
    private Client<ListParameters> listParametersClient;
    private Client<DescribeParameters> describeParametersClient;

    public SyncParametersClient(final Node node, final QoSProfile profileParameter) {
        this(node, null, profileParameter);
    }

    public SyncParametersClient(final Node node, String remoteNodeName, final QoSProfile profileParameter) {
        if (remoteNodeName == null || remoteNodeName.equals("")) {
            remoteNodeName = node.getName();
        }

        try {
            this.getParametersClient = node.<GetParameters>createClient(
                    GetParameters.class,
                    remoteNodeName + "__get_parameters",
                    profileParameter);
            this.getParameterTypesClient = node.<GetParameterTypes>createClient(
                    GetParameterTypes.class,
                    remoteNodeName + "__get_parameter_types",
                    profileParameter);
            this.setParametersClient = node.<SetParameters>createClient(
                    SetParameters.class,
                    remoteNodeName + "__set_parameters",
                    profileParameter);
            this.listParametersClient = node.<ListParameters>createClient(
                    ListParameters.class,
                    remoteNodeName + "__list_parameters",
                    profileParameter);
            this.describeParametersClient = node.<DescribeParameters>createClient(
                    DescribeParameters.class,
                    remoteNodeName + "__describe_parameters",
                    profileParameter);
        } catch (Exception e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    public boolean hasParameter(final String parameterName) {
        return false;
    }

    public List<ParameterVariant<?>> getParameters(final List<String> list) {
        List<ParameterVariant<?>> result = null;

        GetParameters_Request request = new GetParameters_Request();
        request.setNames(list);

     // Call service...
        Future<GetParameters_Response> future = this.getParametersClient.sendRequest(request);

        if (future != null) {
            try {
                List<ParameterValue> values = future.get().getValues();

                result = new ArrayList<ParameterVariant<?>>();
                for (int i = 0; i < values.size(); i++) {
                    ParameterValue parameterValue = values.get(i);

                    Parameter parameter = new Parameter();
                    parameter.setName(request.getNames().get(i));
                    parameter.setValue(parameterValue);

                    ParameterVariant<?> parameterVariant = ParameterVariant.fromParameter(parameter);
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
            System.out.println("add_two_ints_client was interrupted. Exiting.");
        }

        return result;
    }

    public ArrayList<ParameterType> getParameterTypes(final List<String> parameterNames) {
        return null;
    }

    public List<SetParametersResult> setParameters(final List<ParameterVariant<?>> list) {
        List<SetParametersResult> result = null;

        // Set request.
        SetParameters_Request request = new SetParameters_Request();

        List<Parameter> convList = new ArrayList<Parameter>();
        for (ParameterVariant<?> parameterVariant : list) {
            Parameter param = parameterVariant.toParameter();
            convList.add(param);
        }
        request.setParameters(convList);

        // Call service...
        Future<SetParameters_Response> future = this.setParametersClient.sendRequest(request);

        if (future != null) {
            try {
                result = future.get().getResults();
            } catch (InterruptedException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            } catch (ExecutionException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        } else {
            System.out.println("add_two_ints_client was interrupted. Exiting.");
        }
        return result;
    }

    public ListParametersResult listParameters(final List<String> list, int i) {
        // TODO Auto-generated method stub
        return null;
    }

    public Subscription<?> onParameterEvent(final ParameterConsumer parameterConsumer) { // rcl_interfaces.msg.ParameterEvent
        // TODO Auto-generated method stub
        return null;
    }

}
