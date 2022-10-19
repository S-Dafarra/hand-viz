/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef HAND_VIZ_HANDSVISUALIZER_H
#define HAND_VIZ_HANDSVISUALIZER_H

#include <RobotsViz/VtkiCubHand.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Matrix.h>

#include <Eigen/Dense>

#include <Eye.h>

#include <memory>
#include <tuple>

class HandsVisualizer
{

    yarp::dev::PolyDriver       ddtransformclient;
    yarp::dev::IFrameTransform       *iframetrans{nullptr};
    std::shared_ptr<RobotsViz::VtkiCubHand> leftHand;
    std::shared_ptr<RobotsViz::VtkiCubHand> rightHand;
    Eye leftEye;
    Eye rightEye;
    yarp::os::BufferedPort<yarp::sig::FlexImage> leftEyeOutputPort, rightEyeOutputPort;
    Eigen::Matrix4d leftTransform;
    Eigen::Matrix4d rightTransform;
    std::string head_frame;
    std::string left_frame;
    std::string right_frame;
    yarp::sig::Matrix leftTransformYarp, rightTransformYarp;
    Eigen::Matrix4d leftFrameToHand;
    Eigen::Matrix4d rightFrameToHand;
    bool blocking;

    Eigen::Matrix4d toEigen(const yarp::sig::Matrix& input);

public:

    bool configure(const yarp::os::ResourceFinder& rf);

    bool update();
};

#endif // HAND_VIZ_HANDSVISUALIZER_H
