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
import java.util.List;

import org.ros2.rcljava.namespace.GraphName;
import org.ros2.rcljava.node.Node;
import org.ros2.rcljava.node.service.RMWRequestId;
import org.ros2.rcljava.node.service.Service;
import org.ros2.rcljava.node.service.ServiceCallback;
import org.ros2.rcljava.node.topic.Publisher;
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
import rcl_interfaces.srv.SetParametersAtomically;
import rcl_interfaces.srv.SetParametersAtomically_Request;
import rcl_interfaces.srv.SetParametersAtomically_Response;
import rcl_interfaces.srv.SetParameters_Request;
import rcl_interfaces.srv.SetParameters_Response;

/**
 * Parameter Variant.
 *
 */
public class ParameterService {
    private static final Logger logger = LoggerFactory.getLogger(ParameterService.class);

    protected static final String TOPIC_GETPARAMETERS 			= "~/_get_parameters";
    protected static final String TOPIC_GETPARAMETERTYPES 		= "~/_get_parameter_types";
    protected static final String TOPIC_SETPARAMETERS 			= "~/_set_parameters";
    protected static final String TOPIC_SETPARAMETERSATOMICALLY = "~/_set_parameters_atomically";
    protected static final String TOPIC_DESCRIBEPARAMETERS 		= "~/_describe_parameters";
    protected static final String TOPIC_LISTPARAMETERS 			= "~/_list_parameters";

    private final Node ownerNode;
    private final QoSProfile qosProfile;

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
        this.qosProfile = profileParameter;

        try {
            logger.debug("Create Parameters stack " + node.getName());

            this.loadGetParameter();
            this.loadGetParametertypes();
            this.loadSetParameters();
            this.loadSetParametersAtomically();
            this.loadDescribeParameters();
            this.loadListParameters();
            this.loadParametersEvent();

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void notifyAddEvent(final ParameterEvent param) {
        if (this.eventparameterPublisher != null) {
            this.eventparameterPublisher.publish(param);
        }
    }

    /**
     * Safely destroy the underlying ROS2 subscriber structure.
     */
    public void dispose() {
        logger.debug("Destroy parameter client : " + this.ownerNode.getName());

        if (this.getParametersService != null) {
            this.getParametersService.dispose();
        }
        if (this.getParameterTypesService != null) {
            this.getParameterTypesService.dispose();
        }
        if (this.setParametersService != null) {
            this.setParametersService.dispose();
        }
        if (this.setParametersAtomicallyService != null) {
            this.setParametersAtomicallyService.dispose();
        }
        if (this.describeParametersService != null) {
            this.describeParametersService.dispose();
        }
        if (this.listParametersService != null) {
            this.listParametersService.dispose();
        }
        if (this.eventparameterPublisher != null) {
            this.eventparameterPublisher.dispose();
        }
    }

    private void loadGetParameter() {
        final String fqnGetParameter =  GraphName.getFullName(this.ownerNode, TOPIC_GETPARAMETERS, null);
        if (GraphName.isValidTopic(fqnGetParameter)) {
            this.getParametersService = this.ownerNode.<GetParameters>createService(
                    GetParameters.class,
                    fqnGetParameter,
                    new ServiceCallback<GetParameters_Request, GetParameters_Response>() {

                        @Override
                        public void dispatch(
                                final RMWRequestId header,
                                final GetParameters_Request request,
                                final GetParameters_Response response) {
                            logger.debug("Replies to get Parameters.");
                            final List<ParameterValue> paramsResult = new ArrayList<ParameterValue>();

                            final List<ParameterVariant<?>> paramsCurrent = ownerNode.getParameters(request.getNames());
                            for (final ParameterVariant<?> parameterVariant : paramsCurrent) {
                                paramsResult.add(parameterVariant.toParameterValue());
                            }

                            response.setValues(paramsResult);
                        }
                    },
                    this.qosProfile);
        }
    }

    private void loadGetParametertypes() {
        final String fqnGetParametertypes = GraphName.getFullName(this.ownerNode, TOPIC_GETPARAMETERTYPES, null);
        if (GraphName.isValidTopic(fqnGetParametertypes)) {
            this.getParameterTypesService = this.ownerNode.<GetParameterTypes>createService(
                    GetParameterTypes.class,
                    fqnGetParametertypes,
                    new ServiceCallback<GetParameterTypes_Request, GetParameterTypes_Response>() {

                        @Override
                        public void dispatch(
                                final RMWRequestId header,
                                final GetParameterTypes_Request request,
                                final GetParameterTypes_Response response) {
                            logger.debug("Replies to get Parameter Types !");

                            response.setTypes(ownerNode.getParametersTypes(request.getNames()));
                        }
                    },
                    this.qosProfile);
        }
    }

    private void loadSetParameters() {
        final String fqnSetParameters =  GraphName.getFullName(this.ownerNode, TOPIC_SETPARAMETERS, null);
        if (GraphName.isValidTopic(fqnSetParameters)) {
            this.setParametersService = this.ownerNode.<SetParameters>createService(
                    SetParameters.class,
                    fqnSetParameters,
                    new ServiceCallback<SetParameters_Request, SetParameters_Response>() {

                        @Override
                        public void dispatch(
                                final RMWRequestId header,
                                final SetParameters_Request request,
                                final SetParameters_Response response) {

                            logger.debug("Replies to set Parameters.");
                            final List<ParameterVariant<?>> parameterVariants = new ArrayList<ParameterVariant<?>>();
                            for (final Parameter parameterVariant : request.getParameters()) {
                                parameterVariants.add(ParameterVariant.fromParameter(parameterVariant));
                            }
                            final List<SetParametersResult> result = ownerNode.setParameters(parameterVariants);
                            response.setResults(result);
                        }
                    },
                    this.qosProfile);
        }
    }

    private void loadSetParametersAtomically() {
        final String fqnSetParametersAtomically =  GraphName.getFullName(this.ownerNode, TOPIC_SETPARAMETERSATOMICALLY, null);
        if (GraphName.isValidTopic(fqnSetParametersAtomically)) {
            this.setParametersAtomicallyService = this.ownerNode.<SetParametersAtomically>createService(
                    SetParametersAtomically.class,
                    fqnSetParametersAtomically,
                    new ServiceCallback<SetParametersAtomically_Request, SetParametersAtomically_Response>() {

                        @Override
                        public void dispatch(
                                final RMWRequestId header,
                                final SetParametersAtomically_Request request,
                                final SetParametersAtomically_Response response) {

                            logger.debug("Replies to set Parameters Atomically.");
                            final List<ParameterVariant<?>> parameterVariants = new ArrayList<ParameterVariant<?>>();
                            for (final Parameter parameterVariant : request.getParameters()) {
                                parameterVariants.add(ParameterVariant.fromParameter(parameterVariant));
                            }
                            response.setResult(ownerNode.setParametersAtomically(parameterVariants));
                        }
                    },
                    this.qosProfile);
        }
    }

    private void loadDescribeParameters() {
        final String fqnDescribeParameters =  GraphName.getFullName(this.ownerNode, TOPIC_DESCRIBEPARAMETERS, null);
        if (GraphName.isValidTopic(fqnDescribeParameters)) {
            this.describeParametersService = this.ownerNode.<DescribeParameters>createService(
                    DescribeParameters.class,
                    fqnDescribeParameters,
                    new ServiceCallback<DescribeParameters_Request, DescribeParameters_Response>() {

                        @Override
                        public void dispatch(
                                final RMWRequestId header,
                                final DescribeParameters_Request request,
                                final DescribeParameters_Response response) {

                            logger.debug("Replies to describe Parameters. ! NOT IMPLEMENTED !");

                            final List<ParameterDescriptor> listDescritiorResult = new ArrayList<ParameterDescriptor>();

                            final ParameterDescriptor descriptor = new ParameterDescriptor();
//                            descriptor.setName(arg0);
//                            descriptor.setType(arg0);
                            listDescritiorResult.add(descriptor);

                            response.setDescriptors(listDescritiorResult);
                        }
                    },
                    this.qosProfile);
        }
    }

    private void loadListParameters() {
        final String fqnListParameters =  GraphName.getFullName(this.ownerNode, TOPIC_LISTPARAMETERS, null);
        if (GraphName.isValidTopic(fqnListParameters)) {
        this.listParametersService = this.ownerNode.<ListParameters>createService(
                ListParameters.class,
                fqnListParameters,
                new ServiceCallback<ListParameters_Request, ListParameters_Response>() {

                    @Override
                    public void dispatch(
                            final RMWRequestId header,
                            final ListParameters_Request request,
                            final ListParameters_Response response) {

                        logger.debug("Replies to list of Parameters.");
                        final ListParametersResult listParamResult = new ListParametersResult();
                        listParamResult.setNames(ownerNode.getParametersNames());
                        response.setResult(listParamResult);

                    }
                },
                this.qosProfile);
        }
    }

    private void loadParametersEvent() {
        final String fqnParametersEvent =  GraphName.getFullName(this.ownerNode, Topics.PARAM_EVENT, null);
        if (GraphName.isValidTopic(fqnParametersEvent)) {
            this.eventparameterPublisher = this.ownerNode.createPublisher(ParameterEvent.class, fqnParametersEvent, QoSProfile.PARAMETER_EVENTS);
        }
    }

}
