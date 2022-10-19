/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */


#include <HandsVisualizer.h>

#include <yarp/os/LogStream.h>
#include <sstream>

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
    m_settings.parse(rf);
    yInfo() << "Using the following settings:" << m_settings.toString(8);

    yarp::os::Property pTransformclient_cfg;
    pTransformclient_cfg.put("device", "transformClient");
    pTransformclient_cfg.put("local", "/" + m_settings.name + "/transformClient");
    pTransformclient_cfg.put("remote",  m_settings.tfRemote);

    bool ok_client = m_ddtransformclient.open(pTransformclient_cfg);
    if (!ok_client)
    {
        yError()<<"Problem in opening the transformClient device";
        yError()<<"Is the transformServer YARP device running?";
    }
    if (ok_client && !m_ddtransformclient.view(m_iframetrans))
    {
        yError()<<"IFrameTransform I/F is not implemented";
    }

    Eye::Settings leftSettings, rightSettings;
    leftSettings.fps = m_settings.fps;
    leftSettings.width = m_settings.windowWidth;
    leftSettings.height = m_settings.windowHeight;
    leftSettings.blocking = m_settings.blocking;
    leftSettings.backgroundColor = std::tuple<double, double, double>({m_settings.backgroundColor(0), m_settings.backgroundColor(1), m_settings.backgroundColor(2)});
    leftSettings.cameraPosition = m_settings.headToLeftEye;
    leftSettings.forwardDirection = m_settings.forwardDirection;
    leftSettings.upDirection = m_settings.upDirection;
    leftSettings.viewAngle = m_settings.viewAngle;

    rightSettings = leftSettings;
    rightSettings.cameraPosition = m_settings.headToRightEye;


    if (!m_leftEye.initialize(leftSettings))
    {
        return false;
    }

    if (!m_rightEye.initialize(rightSettings))
    {
        return false;
    }

    /* Show hand according to forward kinematics. */

    m_leftHand = std::make_shared<RobotsViz::VtkiCubHand>(m_settings.robot_name, "left", m_settings.name + "/hand_fk/left", m_settings.use_fingers, m_settings.use_analogs,
                                                        std::tuple<double, double, double>({m_settings.handColor(0), m_settings.handColor(1), m_settings.handColor(2)}),
                                                        m_settings.handOpacity, !m_settings.filterAbduction);
    m_rightHand = std::make_shared<RobotsViz::VtkiCubHand>(m_settings.robot_name, "right", m_settings.name + "/hand_fk/right", m_settings.use_fingers, m_settings.use_analogs,
                                                         std::tuple<double, double, double>({m_settings.handColor(0), m_settings.handColor(1), m_settings.handColor(2)}),
                                                         m_settings.handOpacity, !m_settings.filterAbduction);

    if (!m_leftEye.addContents({{"left_hand", m_leftHand}, {"right_hand", m_rightHand}}))
    {
        return false;
    }

    if (!m_rightEye.addContents({{"left_hand", m_leftHand}, {"right_hand", m_rightHand}}))
    {
        return false;
    }

    if (!m_leftEye.prepareVisualization())
    {
        return false;
    }

    if (!m_rightEye.prepareVisualization())
    {
        return false;
    }


    if (!m_leftEyeOutputPort.open("/" + m_settings.name + "/left/image"))
    {
        return false;
    }
    if (!m_rightEyeOutputPort.open("/" + m_settings.name + "/right/image"))
    {
        return false;
    }


    //Initial transforms
    m_leftTransform << 0.0,  0.0,  1.0, 1.0,
                       0.0, -1.0,  0.0, 0.05,
                       1.0,  0.0,  0.0, 0.0,
                       0.0,  0.0,  0.0, 1.0;
    m_leftHand->setTransform(m_leftTransform);

    m_rightTransform << 0.0,  0.0, -1.0,  1.0,
                        0.0,  1.0,  0.0, -0.05,
                        1.0,  0.0,  0.0,  0.0,
                        0.0,  0.0,  0.0,  1.0;
    m_rightHand->setTransform(m_rightTransform);


    m_leftTransformYarp.resize(4,4);
    m_leftTransformYarp.eye();

    m_rightTransformYarp.resize(4,4);
    m_rightTransformYarp.eye();

    return true;
}

bool HandsVisualizer::update()
{
    if (m_iframetrans)
    {
        if (m_iframetrans->canTransform(m_settings.left_frame, m_settings.head_frame))
        {
            if (m_iframetrans->getTransform(m_settings.left_frame, m_settings.head_frame, m_leftTransformYarp))
            {
                m_leftTransform = toEigen(m_leftTransformYarp) * m_settings.leftFrameToHand;
            }
        }

        if (m_iframetrans->canTransform(m_settings.right_frame, m_settings.head_frame))
        {
            if (m_iframetrans->getTransform(m_settings.right_frame, m_settings.head_frame, m_rightTransformYarp))
            {
                m_rightTransform = toEigen(m_rightTransformYarp) * m_settings.rightFrameToHand;
            }
        }
    }

    m_leftHand->setTransform(m_leftTransform);
    m_rightHand->setTransform(m_rightTransform);

    m_leftHand->update(m_settings.blocking);
    m_rightHand->update(m_settings.blocking);

    m_leftEye.render();
    m_rightEye.render();
    m_leftEye.takeScreenshot();
    m_rightEye.takeScreenshot();

    yarp::sig::FlexImage& imageLeftToBeSent = m_leftEyeOutputPort.prepare();
    imageLeftToBeSent.setPixelCode(m_leftEye.screenshot().getPixelCode());
    imageLeftToBeSent.setExternal(m_leftEye.screenshot().getRawImage(), m_leftEye.screenshot().width(), m_leftEye.screenshot().height()); //Avoid to copy


    yarp::sig::FlexImage& imageRightToBeSent = m_rightEyeOutputPort.prepare();
    imageRightToBeSent.setPixelCode(m_rightEye.screenshot().getPixelCode());
    imageRightToBeSent.setExternal(m_rightEye.screenshot().getRawImage(), m_rightEye.screenshot().width(), m_rightEye.screenshot().height()); //Avoid to copy

    m_leftEyeOutputPort.write();
    m_rightEyeOutputPort.write();

    return true;
}

Eigen::Vector3d HandsVisualizer::Settings::parse3DVector(const yarp::os::Searchable &rf, const std::string &key, const Eigen::Vector3d &defaultValue)
{
    if (!rf.check(key))
    {
        return defaultValue;
    }

    yarp::os::Value& value = rf.find(key);

    if (!value.isList())
    {
        yWarning() << key << "found in the configuration parameters, but it is not a list. Using default value.";
        return defaultValue;
    }

    yarp::os::Bottle* list = value.asList();

    if (list->size() != 3)
    {
        yWarning() << key << "found in the configuration parameters, but it is not a list with three elements. Using default value.";
        return defaultValue;
    }

    Eigen::Vector3d output;

    for (size_t i = 0; i < 3; ++i)
    {
        if (!list->get(i).isFloat64() && !list->get(i).isInt32() && !list->get(i).isInt64())
        {
            yWarning() << "The element of" << key << "at position" << i << "is not numeric. Using default value.";
            return defaultValue;
        }
        output(i) = list->get(i).asFloat64();
    }

    return output;
}

Eigen::Quaterniond HandsVisualizer::Settings::parseQuaternion(const yarp::os::Searchable &rf, const std::string &key, const Eigen::Quaterniond &defaultValue)
{
    if (!rf.check(key))
    {
        return defaultValue;
    }

    yarp::os::Value& value = rf.find(key);

    if (!value.isList())
    {
        yWarning() << key << "found in the configuration parameters, but it is not a list. Using default value.";
        return defaultValue;
    }

    yarp::os::Bottle* list = value.asList();

    if (list->size() != 4)
    {
        yWarning() << key << "found in the configuration parameters, but it is not a list with four elements. Using default value.";
        return defaultValue;
    }

    Eigen::Quaterniond output;

    for (size_t i = 0; i < 4; ++i)
    {
        if (!list->get(i).isFloat64() && !list->get(i).isInt32() && !list->get(i).isInt64())
        {
            yWarning() << "The element of" << key << "at position" << i << "is not numeric. Using default value.";
            return defaultValue;
        }
    }

    output.w() = list->get(0).asFloat64();
    output.x() = list->get(1).asFloat64();
    output.y() = list->get(2).asFloat64();
    output.z() = list->get(3).asFloat64();

    return output;
}

void HandsVisualizer::Settings::parse(const yarp::os::Searchable &rf)
{
    robot_name = rf.check("robot", yarp::os::Value("icub")).asString();
    name = rf.check("name", yarp::os::Value("hand-visualizer")).asString();
    blocking = rf.check("blocking") && (rf.find("blocking").isNull() || rf.find("blocking").asBool());
    use_fingers = rf.check("use_fingers") && (rf.find("use_fingers").isNull() || rf.find("use_fingers").asBool());
    use_analogs = rf.check("use_analogs") && (rf.find("use_analogs").isNull() || rf.find("use_analogs").asBool());
    filterAbduction = rf.check("filter_abduction") && (rf.find("filter_abduction").isNull() || rf.find("filter_abduction").asBool());
    head_frame = rf.check("head_frame", yarp::os::Value("head")).asString();
    left_frame = rf.check("left_frame", yarp::os::Value("l_hand")).asString();
    right_frame = rf.check("right_frame", yarp::os::Value("r_hand")).asString();
    viewAngle = rf.check("view_angle", yarp::os::Value(85.0)).asFloat64();
    fps = rf.check("desired_fps", yarp::os::Value(30.0)).asFloat64();
    handOpacity = rf.check("hand_opacity", yarp::os::Value(1.0)).asFloat64();
    handColor = parse3DVector(rf, "hand_color", {0.4, 0.6, 1.0});
    backgroundColor = parse3DVector(rf, "background_color", {0.0, 0.0, 0.0});
    windowWidth = rf.check("window_width", yarp::os::Value(600)).asInt32();
    windowHeight = rf.check("window_height", yarp::os::Value(600)).asInt32();
    tfRemote = rf.check("tf_remote", yarp::os::Value("/transformServer")).asString();
    headToLeftEye = parse3DVector(rf, "head_to_left_eye_offset", {0.051, 0.034, 0.013});
    headToRightEye = parse3DVector(rf, "head_to_right_eye_offset", {0.051, -0.034, 0.013});
    forwardDirection = parse3DVector(rf, "forward_direction", Eigen::Vector3d::UnitX());
    upDirection = parse3DVector(rf, "up_direction", Eigen::Vector3d::UnitZ());

    leftFrameToHand.setIdentity();
    rightFrameToHand.setIdentity();
    leftFrameToHand.block<3,1>(0, 3) = parse3DVector(rf, "left_frame_to_hand_offset", {-0.002, -0.018, -0.059});
    rightFrameToHand.block<3,1>(0, 3) = parse3DVector(rf, "right_frame_to_hand_offset", {-0.002, 0.018, -0.059});
    leftFrameToHand.block<3,3>(0, 0) = parseQuaternion(rf, "left_frame_to_hand_quaternion_wxyz", Eigen::Quaterniond(-0.5, 0.5, -0.5, -0.5)).matrix();
    rightFrameToHand.block<3,3>(0, 0) = parseQuaternion(rf, "right_frame_to_hand_quaternion_wxyz", Eigen::Quaterniond(-0.5, 0.5, -0.5, -0.5)).matrix();
}

std::string HandsVisualizer::Settings::toString(size_t indentation)
{
    auto vecToString = [](const Eigen::Vector3d& input) -> std::string
    {
        std::stringstream output;
        output << "\"(" << input(0) << ", " << input(1) << ", " << input(2) << ")\"";
        return output.str();
    };

    auto quatToString = [](const Eigen::Quaterniond& input) -> std::string
    {
        std::stringstream output;
        output << "\"(" << input.w() << ", " << input.x() << ", " << input.y() << ", " << input.z() << ")\"";
        return output.str();
    };

    std::string indentationString(indentation, ' ');


    std::stringstream optionsOut;
    optionsOut << std::endl;
    optionsOut << indentationString << "name " << name << std::endl;
    optionsOut << indentationString << "robot " << robot_name << std::endl;
    optionsOut << indentationString << "blocking " << blocking << std::endl;
    optionsOut << indentationString << "use_fingers " << use_fingers << std::endl;
    optionsOut << indentationString << "use_analogs " << use_analogs << std::endl;
    optionsOut << indentationString << "filter_abduction " << filterAbduction << std::endl;
    optionsOut << indentationString << "head_frame " << head_frame << std::endl;
    optionsOut << indentationString << "left_frame " << left_frame << std::endl;
    optionsOut << indentationString << "right_frame " << right_frame << std::endl;
    optionsOut << indentationString << "view_angle " << viewAngle << std::endl;
    optionsOut << indentationString << "desired_fps " << fps << std::endl;
    optionsOut << indentationString << "hand_opacity " << handOpacity << std::endl;
    optionsOut << indentationString << "hand_color " << vecToString(handColor) << std::endl;
    optionsOut << indentationString << "background_color " << vecToString(backgroundColor) << std::endl;
    optionsOut << indentationString << "window_width " << windowWidth << std::endl;
    optionsOut << indentationString << "window_height " << windowHeight << std::endl;
    optionsOut << indentationString << "tf_remote " << tfRemote << std::endl;
    optionsOut << indentationString << "head_to_left_eye_offset " << vecToString(headToLeftEye) << std::endl;
    optionsOut << indentationString << "head_to_right_eye_offset " << vecToString(headToRightEye) << std::endl;
    optionsOut << indentationString << "forward_direction " << vecToString(forwardDirection) << std::endl;
    optionsOut << indentationString << "up_direction " << vecToString(upDirection) << std::endl;
    optionsOut << indentationString << "left_frame_to_hand_offset " << vecToString(leftFrameToHand.block<3,1>(0, 3)) << std::endl;
    optionsOut << indentationString << "right_frame_to_hand_offset " << vecToString(rightFrameToHand.block<3,1>(0, 3)) << std::endl;
    optionsOut << indentationString << "left_frame_to_hand_quaternion_wxyz " << quatToString(Eigen::Quaterniond(leftFrameToHand.block<3,3>(0, 0))) << std::endl;
    optionsOut << indentationString << "right_frame_to_hand_quaternion_wxyz " << quatToString(Eigen::Quaterniond(rightFrameToHand.block<3,3>(0, 0))) << std::endl;
    return optionsOut.str();
}
