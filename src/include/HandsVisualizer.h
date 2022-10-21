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
#include <thread>
#include <atomic>

#include <thrifts/HandVisualizerCommands.h>

class HandsVisualizer : public HandVisualizerCommands
{

    class Settings
    {
        Eigen::Vector3d parse3DVector(const yarp::os::Searchable& rf, const std::string& key, const Eigen::Vector3d& defaultValue);

        Eigen::Quaterniond parseQuaternion(const yarp::os::Searchable& rf, const std::string& key, const Eigen::Quaterniond& defaultValue);

    public:
        std::mutex mutex;
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
    yarp::os::Port m_rpcPort;
    Settings m_settings;
    std::mutex m_handsMutex, m_eyesMutex;
    yarp::os::ConnectionReader* m_tempReader;

    Eigen::Matrix4d toEigen(const yarp::sig::Matrix& input);

public:

    bool configure(const yarp::os::ResourceFinder& rf);

    bool update();

    void close();

    /**
     * Set the vertical view angle of the camera (in degrees).
     * @return true/false in case of success/failure.
     */
    virtual bool setViewAngle(const double angleInDeg) override;

    /**
     * Set the color of the hand. The values are supposed to be between 0 and 1.
     * @return true/false in case of success/failure.
     */
    virtual bool setHandColor(const double r, const double g, const double b) override;

    /**
     * Set the opacity of the hand.
     * 0 is fully transparent, 1 is fully opaque.
     * @return true/false in case of success/failure.
     */
    virtual bool setHandOpacity(const double opacity) override;

    /**
     * Set the position of the left eye with respect the head frame.
     * The components are expressed in meters.
     * @return true/false in case of success/failure.
     */
    virtual bool setHeadToLeftEyeOffset(const double x, const double y, const double z) override;

    /**
     * Set the position of the right eye with respect the head frame.
     * The components are expressed in meters.
     * @return true/false in case of success/failure.
     */
    virtual bool setHeadToRightEyeOffset(const double x, const double y, const double z) override;

    /**
     * Set the position of the left hand with respect the left frame.
     * The components are expressed in meters.
     * @return true/false in case of success/failure.
     */
    virtual bool setLeftFrameToHandOffset(const double x, const double y, const double z) override;

    /**
     * Set the position of the right hand with respect the right frame.
     * The components are expressed in meters.
     * @return true/false in case of success/failure.
     */
    virtual bool setRightFrameToHandOffset(const double x, const double y, const double z) override;

    /**
     * Set the orientation of the left hand with respect the left frame.
     * @return true/false in case of success/failure.
     */
    virtual bool setLeftFrameToHandQuaternion(const double w, const double x, const double y, const double z) override;

    /**
     * Set the orientation of the right hand with respect the right frame.
     * @return true/false in case of success/failure.
     */
    virtual bool setRightFrameToHandQuaternion(const double w, const double x, const double y, const double z) override;

    /**
     * Prints the settings
     * @return A string that can be copied in a configuration file,
     *         including the modifications set from RPC
     */
    virtual std::string printSettings() override;

    //This is to fix the print of settings
    bool read(yarp::os::ConnectionReader& connection) override;
};

#endif // HAND_VIZ_HANDSVISUALIZER_H
