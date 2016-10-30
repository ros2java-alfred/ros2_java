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
import java.util.Collection;
import java.util.List;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import org.ros2.rcljava.qos.QoSProfile;
import org.ros2.rcljava.RMWRequestId;
import org.ros2.rcljava.node.Node;
import org.ros2.rcljava.node.service.Client;
import org.ros2.rcljava.node.service.Service;
import org.ros2.rcljava.node.service.TriConsumer;
import org.ros2.rcljava.node.topic.Publisher;

import rcl_interfaces.msg.ListParametersResult;
import rcl_interfaces.msg.Parameter;
import rcl_interfaces.msg.ParameterDescriptor;
import rcl_interfaces.msg.ParameterValue;
import rcl_interfaces.msg.ParameterEvent;

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
 * Parameter Variant.
 *
 * @author Mickael Gaillard <mick.gaillard@gmail.com>
 */
@SuppressWarnings("unused")
public class ParameterService {
    private static final Logger logger = LoggerFactory.getLogger(ParameterService.class);

    private Service<GetParameters> getParametersService;
    private Service<GetParameterTypes> getParameterTypesService;
    private Service<SetParameters> setParametersService;
    private Service<ListParameters> listParametersService;
    private Service<DescribeParameters> describeParametersService;
    private Publisher<ParameterEvent> eventparameterPublisher;

    public ParameterService(final Node node, final QoSProfile profileParameter) {

        try {
            logger.debug("Create Parameters stack " + node.getName());

            this.getParametersService = node.<GetParameters>createService(
                    GetParameters.class,
                    node.getName() + "__get_parameters",
                    new TriConsumer<RMWRequestId, GetParameters_Request, GetParameters_Response>() {

                        @Override
                        public void accept(
                                final RMWRequestId header,
                                final GetParameters_Request request,
                                final GetParameters_Response response) {
                            logger.debug("Replies to get Parameters.");
                            List<ParameterValue> paramsResult = new ArrayList<ParameterValue>();

                            List<ParameterVariant<?>> paramsCurrent = node.getParameters(request.getNames());
                            for (ParameterVariant<?> parameterVariant : paramsCurrent) {
                                paramsResult.add(parameterVariant.toParameterValue());
                            }

                            response.setValues(paramsResult);
                        }
                    });

            this.getParameterTypesService = node.<GetParameterTypes>createService(
                    GetParameterTypes.class,
                    node.getName() + "__get_parameter_types",
                    new TriConsumer<RMWRequestId, GetParameterTypes_Request, GetParameterTypes_Response>() {

                        @Override
                        public void accept(
                                final RMWRequestId header,
                                final GetParameterTypes_Request request,
                                final GetParameterTypes_Response response) {

                            logger.debug("Replies to get Parameter Types ! NOT IMPLEMENTED !");
                        }
                    });

            this.setParametersService = node.<SetParameters>createService(
                    SetParameters.class,
                    node.getName() + "__set_parameters",
                    new TriConsumer<RMWRequestId, SetParameters_Request, SetParameters_Response>() {

                        @Override
                        public void accept(
                                final RMWRequestId header,
                                final SetParameters_Request request,
                                final SetParameters_Response response) {

                            logger.debug("Replies to set Parameters.");
                            List<ParameterVariant<?>> parameterVariants = new ArrayList<ParameterVariant<?>>();
                            for (Parameter parameterVariant : request.getParameters()) {
                                parameterVariants.add(ParameterVariant.fromParameter(parameterVariant));
                            }
                            response.setResults(node.setParameters(parameterVariants));
                        }
                    });

            this.listParametersService = node.<ListParameters>createService(
                    ListParameters.class,
                    node.getName() + "__list_parameters",
                    new TriConsumer<RMWRequestId, ListParameters_Request, ListParameters_Response>() {

                        @Override
                        public void accept(
                                final RMWRequestId header,
                                final ListParameters_Request request,
                                final ListParameters_Response response) {

                            logger.debug("Replies to list of Parameters.");
                            ListParametersResult listParamResult = new ListParametersResult();
                            listParamResult.setNames(node.getParametersNames());
                            response.setResult(listParamResult);

                        }
                    });

            this.describeParametersService = node.<DescribeParameters>createService(
                    DescribeParameters.class,
                    node.getName() + "__describe_parameters",
                    new TriConsumer<RMWRequestId, DescribeParameters_Request, DescribeParameters_Response>() {

                        @Override
                        public void accept(
                                final RMWRequestId header,
                                final DescribeParameters_Request request,
                                final DescribeParameters_Response response) {

                            logger.debug("Replies to describe Parameters. ! NOT IMPLEMENTED !");
//                            request.getNames()
//
//
//                            List<ParameterDescriptor> listDescritiorResult = null;
//
//                            ParameterDescriptor descriptor = new ParameterDescriptor();
//                            descriptor.setName(arg0);
//                            descriptor.setType(arg0);
//
//                            response.setDescriptors(listDescritiorResult);
                        }
                    });

            this.eventparameterPublisher = node.createPublisher(ParameterEvent.class, "parameter_events", profileParameter);

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private void notifyAddEvent(Collection<Parameter> param) {
        ParameterEvent eventMsg = new ParameterEvent();
        eventMsg.setNewParameters(param);
        this.eventparameterPublisher.publish(eventMsg);
    }

}
