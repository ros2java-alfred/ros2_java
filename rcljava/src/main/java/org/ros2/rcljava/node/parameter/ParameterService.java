/* Copyright 2016 Esteve Fernandez <esteve@apache.org>
 * Copyright 2016 Mickael Gaillard <mick.gaillard@gmail.com>
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

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import org.ros2.rcljava.qos.QoSProfile;
import org.ros2.rcljava.node.Node;
import org.ros2.rcljava.node.service.RMWRequestId;
import org.ros2.rcljava.node.service.Service;
import org.ros2.rcljava.node.service.TriConsumer;
import org.ros2.rcljava.node.topic.Publisher;
import org.ros2.rcljava.node.topic.Topics;

import rcl_interfaces.msg.ListParametersResult;
import rcl_interfaces.msg.Parameter;
import rcl_interfaces.msg.ParameterDescriptor;
import rcl_interfaces.msg.ParameterValue;
import rcl_interfaces.msg.SetParametersResult;
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
import rcl_interfaces.srv.SetParametersAtomically;
import rcl_interfaces.srv.SetParametersAtomically_Request;
import rcl_interfaces.srv.SetParametersAtomically_Response;

/**
 * Parameter Variant.
 *
 */
public class ParameterService {
    private static final Logger logger = LoggerFactory.getLogger(ParameterService.class);

    protected static final String TOPIC_GETPARAMETERS = "%s__get_parameters";
    protected static final String TOPIC_GETPARAMETERTYPES = "%s__get_parameter_types";
    protected static final String TOPIC_SETPARAMETERS = "%s__set_parameters";
    protected static final String TOPIC_SETPARAMETERSATOMICALLY = "%s__set_parameters_atomically";
    protected static final String TOPIC_DESCRIBEPARAMETERS = "%s__describe_parameters";
    protected static final String TOPIC_LISTPARAMETERS = "%s__list_parameters";


    private final Node ownerNode;

    private Service<GetParameters> getParametersService;
    private Service<GetParameterTypes> getParameterTypesService;
    private Service<SetParameters> setParametersService;
    private Service<SetParametersAtomically> setParametersAtomicallyService;
    private Service<DescribeParameters> describeParametersService;
    private Service<ListParameters> listParametersService;
    private Publisher<ParameterEvent> eventparameterPublisher;

    public ParameterService(final Node node) {
        this(node, QoSProfile.PARAMETER);
    }

    public ParameterService(final Node node, final QoSProfile profileParameter) {
        this.ownerNode = node;

        try {
            logger.debug("Create Parameters stack " + node.getName());

            this.getParametersService = node.<GetParameters>createService(
                    GetParameters.class,
                    String.format(TOPIC_GETPARAMETERS, node.getName()),
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
                    },
                    profileParameter);

            this.getParameterTypesService = node.<GetParameterTypes>createService(
                    GetParameterTypes.class,
                    String.format(TOPIC_GETPARAMETERTYPES, node.getName()),
                    new TriConsumer<RMWRequestId, GetParameterTypes_Request, GetParameterTypes_Response>() {

                        @Override
                        public void accept(
                                final RMWRequestId header,
                                final GetParameterTypes_Request request,
                                final GetParameterTypes_Response response) {
                            logger.debug("Replies to get Parameter Types !");

                            response.setTypes(node.getParametersTypes(request.getNames()));
                        }
                    },
                    profileParameter);

            this.setParametersService = node.<SetParameters>createService(
                    SetParameters.class,
                    String.format(TOPIC_SETPARAMETERS, node.getName()),
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
                            List<SetParametersResult> result = node.setParameters(parameterVariants);
                            response.setResults(result);
                        }
                    },
                    profileParameter);

            this.setParametersAtomicallyService = node.<SetParametersAtomically>createService(
                    SetParametersAtomically.class,
                    String.format(TOPIC_SETPARAMETERSATOMICALLY, node.getName()),
                    new TriConsumer<RMWRequestId, SetParametersAtomically_Request, SetParametersAtomically_Response>() {

                        @Override
                        public void accept(
                                final RMWRequestId header,
                                final SetParametersAtomically_Request request,
                                final SetParametersAtomically_Response response) {

                            logger.debug("Replies to set Parameters Atomically.");
                            List<ParameterVariant<?>> parameterVariants = new ArrayList<ParameterVariant<?>>();
                            for (Parameter parameterVariant : request.getParameters()) {
                                parameterVariants.add(ParameterVariant.fromParameter(parameterVariant));
                            }
                            response.setResult(node.setParametersAtomically(parameterVariants));
                        }
                    },
                    profileParameter);

            this.describeParametersService = node.<DescribeParameters>createService(
                    DescribeParameters.class,
                    String.format(TOPIC_DESCRIBEPARAMETERS, node.getName()),
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
                    },
                    profileParameter);

            this.listParametersService = node.<ListParameters>createService(
                    ListParameters.class,
                    String.format(TOPIC_LISTPARAMETERS, node.getName()),
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
                    },
                    profileParameter);

            this.eventparameterPublisher = node.createPublisher(ParameterEvent.class, Topics.PARAM_EVENT, QoSProfile.PARAMETER_EVENTS);

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void notifyAddEvent(final ParameterEvent param) {
        this.eventparameterPublisher.publish(param);
    }

    /**
     * Safely destroy the underlying ROS2 subscriber structure.
     */
    public void dispose() {
        logger.debug("Destroy parameter client : " + this.ownerNode.getName());
        this.getParametersService.dispose();
        this.getParameterTypesService.dispose();
        this.setParametersService.dispose();
        this.setParametersAtomicallyService.dispose();
        this.describeParametersService.dispose();
        this.listParametersService.dispose();

        this.eventparameterPublisher.dispose();
    }
}
