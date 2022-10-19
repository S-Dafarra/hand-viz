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

    class Settings
    {
        Eigen::Vector3d parse3DVector(const yarp::os::Searchable& rf, const std::string& key, const Eigen::Vector3d& defaultValue);

        Eigen::Quaterniond parseQuaternion(const yarp::os::Searchable& rf, const std::string& key, const Eigen::Quaterniond& defaultValue);

    public:
        std::string robot_name;
        std::string name;
        bool blocking;
        bool use_fingers;
        bool use_analogs;
        bool filterAbduction;
        std::string head_frame;
        std::string left_frame;
        std::string right_frame;
        double viewAngle;
        double fps;
        double handOpacity;
        Eigen::Vector3d handColor;
        Eigen::Vector3d backgroundColor;
        int windowWidth;
        int windowHeight;
        std::string tfRemote;
        Eigen::Vector3d headToLeftEye;
        Eigen::Vector3d headToRightEye;
        Eigen::Vector3d forwardDirection;
        Eigen::Vector3d upDirection;
        Eigen::Matrix4d leftFrameToHand;
        Eigen::Matrix4d rightFrameToHand;

        void parse(const yarp::os::Searchable& rf);

        std::string toString(size_t indentation);
    };

    yarp::dev::PolyDriver m_ddtransformclient;
    yarp::dev::IFrameTransform *m_iframetrans{nullptr};
    std::shared_ptr<RobotsViz::VtkiCubHand> m_leftHand;
    std::shared_ptr<RobotsViz::VtkiCubHand> m_rightHand;
    Eye m_leftEye;
    Eye m_rightEye;
    yarp::os::BufferedPort<yarp::sig::FlexImage> m_leftEyeOutputPort, m_rightEyeOutputPort;
    Eigen::Matrix4d m_leftTransform;
    Eigen::Matrix4d m_rightTransform;
    yarp::sig::Matrix m_leftTransformYarp, m_rightTransformYarp;
    Settings m_settings;

    Eigen::Matrix4d toEigen(const yarp::sig::Matrix& input);

public:

    bool configure(const yarp::os::ResourceFinder& rf);

    bool update();
};

#endif // HAND_VIZ_HANDSVISUALIZER_H
