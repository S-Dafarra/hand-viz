/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */


#include <HandsVisualizer.h>

#include <yarp/os/LogStream.h>

Eigen::Matrix4d HandsVisualizer::toEigen(const yarp::sig::Matrix &input)
{
    if (input.rows() != 4 && input.cols() != 4)
    {
        yError() << "The input yarp matrix is not a 4 by 4.";
        return Eigen::Matrix4d::Identity();
    }

    Eigen::Matrix4d output;

    for (size_t i = 0; i < 4; ++i)
    {
        for (size_t j = 0; j < 4; ++j)
        {
            output(i,j) = input[i][j];
        }
    }

    return output;
}

bool HandsVisualizer::configure(const yarp::os::ResourceFinder &rf)
{
    //------------Parameters---------------
    std::string robot_name = "icubSim";
    std::string name = "hand-visualizer";
    blocking = false;
    bool use_fingers = true;
    bool use_analogs = false;
    bool useAbduction = false;
    head_frame = "head";
    left_frame = "l_hand";
    right_frame = "r_hand";

    Eigen::Vector3d headToLeftEye;
    headToLeftEye << 0.051, 0.034, 0.013;

    Eigen::Vector3d headToRightEye;
    headToRightEye <<  0.051, -0.034, 0.013;

    //    leftFrameToHand << 0.0, -1.0,  0.0, -0.002,
    //                       0.0,  0.0,  1.0, -0.018,
    //                      -1.0,  0.0,  0.0, -0.059,
    //                       0.0,  0.0,  0.0,  1.0;
    leftFrameToHand << 0.0, -1.0,  0.0,  0.015,
            0.0,  0.0,  1.0, -0.018,
            -1.0,  0.0,  0.0, -0.055,
            0.0,  0.0,  0.0,  1.0;
    //    leftFrameToHand.block<3,3>(0,0) = Eigen::AngleAxisd(-0.25, Eigen::Vector3d::UnitY()) * leftFrameToHand.block<3,3>(0,0);


    //    rightFrameToHand << 0.0, -1.0,  0.0, -0.002,
    //                        0.0,  0.0,  1.0,  0.018,
    //                       -1.0,  0.0,  0.0, -0.059,
    //                        0.0,  0.0,  0.0,  1.0;
    rightFrameToHand << 0.0, -1.0,  0.0,  0.005,
            0.0,  0.0,  1.0,  0.010,
            -1.0,  0.0,  0.0, -0.050,
            0.0,  0.0,  0.0,  1.0;
    //    rightFrameToHand.block<3,3>(0,0) = Eigen::AngleAxisd(-0.25, Eigen::Vector3d::UnitY()) * rightFrameToHand.block<3,3>(0,0);

    double viewAngle = 37;

    double fps = 60.0;

    Eigen::Vector3d forwardDirection = Eigen::Vector3d::UnitX();
    Eigen::Vector3d upDirection = Eigen::Vector3d::UnitZ();


    std::tuple<double, double, double> handColor{100.0 / 255.0, 160 / 255.0, 255.0 / 255.0};
    double handOpacity = 0.2;

    int windowWidth = 320;
    int windowHeight = 240;

    std::string tfRemote = "/transformServer";

    //------------------------------------------


    yarp::os::Property pTransformclient_cfg;
    pTransformclient_cfg.put("device", "transformClient");
    pTransformclient_cfg.put("local", "/" + name + "/transformClient");
    pTransformclient_cfg.put("remote",  tfRemote);

    bool ok_client = ddtransformclient.open(pTransformclient_cfg);
    if (!ok_client)
    {
        yError()<<"Problem in opening the transformClient device";
        yError()<<"Is the transformServer YARP device running?";
    }
    if (ok_client && !ddtransformclient.view(iframetrans))
    {
        yError()<<"IFrameTransform I/F is not implemented";
    }

    Eye::Settings leftSettings, rightSettings;
    leftSettings.fps = fps;
    leftSettings.width = windowWidth;
    leftSettings.height = windowHeight;
    leftSettings.blocking = blocking;
    leftSettings.backgroundColor = {0,0,0};
    leftSettings.cameraPosition = headToLeftEye;
    leftSettings.forwardDirection = forwardDirection;
    leftSettings.upDirection = upDirection;
    leftSettings.viewAngle = viewAngle;

    rightSettings = leftSettings;
    rightSettings.cameraPosition = headToRightEye;


    if (!leftEye.initialize(leftSettings))
    {
        return false;
    }

    if (!rightEye.initialize(rightSettings))
    {
        return false;
    }

    /* Show hand according to forward kinematics. */

    leftHand = std::make_shared<RobotsViz::VtkiCubHand>(robot_name, "left", name + "/hand_fk/left", use_fingers, use_analogs, handColor, handOpacity, useAbduction);
    rightHand = std::make_shared<RobotsViz::VtkiCubHand>(robot_name, "right", name + "/hand_fk/right", use_fingers, use_analogs, handColor, handOpacity, useAbduction);

    if (!leftEye.addContents({{"left_hand", leftHand}, {"right_hand", rightHand}}))
    {
        return false;
    }

    if (!rightEye.addContents({{"left_hand", leftHand}, {"right_hand", rightHand}}))
    {
        return false;
    }

    if (!leftEye.prepareVisualization())
    {
        return false;
    }

    if (!rightEye.prepareVisualization())
    {
        return false;
    }


    if (!leftEyeOutputPort.open("/" + name + "/left/image"))
    {
        return false;
    }
    if (!rightEyeOutputPort.open("/" + name + "/right/image"))
    {
        return false;
    }


    //Initial transforms
    leftTransform << 0.0,  0.0,  1.0, 1.0,
            0.0, -1.0,  0.0, 0.05,
            1.0,  0.0,  0.0, 0.0,
            0.0,  0.0,  0.0, 1.0;

    leftHand->setTransform(leftTransform);

    rightTransform << 0.0,  0.0, -1.0,  1.0,
            0.0,  1.0,  0.0, -0.05,
            1.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  1.0;
    rightHand->setTransform(rightTransform);


    leftTransformYarp.resize(4,4);
    leftTransformYarp.eye();

    rightTransformYarp.resize(4,4);
    rightTransformYarp.eye();

    return true;
}

bool HandsVisualizer::update()
{
    if (iframetrans)
    {
        if (iframetrans->canTransform(left_frame, head_frame))
        {
            if (iframetrans->getTransform(left_frame, head_frame, leftTransformYarp))
            {
                leftTransform = toEigen(leftTransformYarp) * leftFrameToHand;
            }
        }

        if (iframetrans->canTransform(right_frame, head_frame))
        {
            if (iframetrans->getTransform(right_frame, head_frame, rightTransformYarp))
            {
                rightTransform = toEigen(rightTransformYarp) * rightFrameToHand;
            }
        }
    }

    leftHand->setTransform(leftTransform);
    rightHand->setTransform(rightTransform);

    leftHand->update(blocking);
    rightHand->update(blocking);

    leftEye.render();
    rightEye.render();
    leftEye.takeScreenshot();
    rightEye.takeScreenshot();

    yarp::sig::FlexImage& imageLeftToBeSent = leftEyeOutputPort.prepare();
    imageLeftToBeSent.setPixelCode(leftEye.screenshot().getPixelCode());
    imageLeftToBeSent.setExternal(leftEye.screenshot().getRawImage(), leftEye.screenshot().width(), leftEye.screenshot().height()); //Avoid to copy


    yarp::sig::FlexImage& imageRightToBeSent = rightEyeOutputPort.prepare();
    imageRightToBeSent.setPixelCode(rightEye.screenshot().getPixelCode());
    imageRightToBeSent.setExternal(rightEye.screenshot().getRawImage(), rightEye.screenshot().width(), rightEye.screenshot().height()); //Avoid to copy

    leftEyeOutputPort.write();
    rightEyeOutputPort.write();

    return true;
}
