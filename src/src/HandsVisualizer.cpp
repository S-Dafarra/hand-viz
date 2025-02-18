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
    leftSettings.offscreen = m_settings.offscreen;

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

    yarp::sig::Matrix leftAnalogBounds;
    leftAnalogBounds.resize(15,2);
    leftAnalogBounds.zero();
    leftAnalogBounds.setCol(0, m_settings.left_analog_upper_bounds);
    leftAnalogBounds.setCol(1, m_settings.left_analog_lower_bounds);

    yarp::sig::Matrix rightAnalogBounds;
    rightAnalogBounds.resize(15,2);
    rightAnalogBounds.zero();
    rightAnalogBounds.setCol(0, m_settings.right_analog_upper_bounds);
    rightAnalogBounds.setCol(1, m_settings.right_analog_lower_bounds);

    m_leftHand = std::make_shared<RobotsViz::VtkiCubHand>(m_settings.robot_name, "left", m_settings.name + "/hand_fk/left", m_settings.use_fingers, m_settings.use_analogs,
                                                        std::tuple<double, double, double>({m_settings.handColor(0), m_settings.handColor(1), m_settings.handColor(2)}),
                                                        m_settings.handOpacity, m_settings.use_analogs_bounds, leftAnalogBounds, !m_settings.filterAbduction);
    m_rightHand = std::make_shared<RobotsViz::VtkiCubHand>(m_settings.robot_name, "right", m_settings.name + "/hand_fk/right", m_settings.use_fingers, m_settings.use_analogs,
                                                         std::tuple<double, double, double>({m_settings.handColor(0), m_settings.handColor(1), m_settings.handColor(2)}),
                                                         m_settings.handOpacity, m_settings.use_analogs_bounds, rightAnalogBounds, !m_settings.filterAbduction);

    m_leftForearmTrasform = std::make_shared<RobotsIO::Utils::ManualTransform>();
    m_leftForearm = std::make_shared<RobotsViz::VtkObject>(RobotsViz::MeshResources("sim_icub3_l_forearm_prt.obj"), m_leftForearmTrasform,
                                                                                            std::tuple<double, double, double>({m_settings.handColor(0), m_settings.handColor(1), m_settings.handColor(2)}),
                                                                                            m_settings.handOpacity);

    m_rightForearmTrasform = std::make_shared<RobotsIO::Utils::ManualTransform>();
    m_rightForearm = std::make_shared<RobotsViz::VtkObject>(RobotsViz::MeshResources("sim_icub3_r_forearm_prt.obj"), m_rightForearmTrasform,
                                                                                            std::tuple<double, double, double>({m_settings.handColor(0), m_settings.handColor(1), m_settings.handColor(2)}),
                                                                                            m_settings.handOpacity);

    if (!m_leftEye.addContents({{"left_hand", m_leftHand}, {"right_hand", m_rightHand}, {"l_forearm", m_leftForearm}, {"r_forearm", m_rightForearm}}))
    {
        return false;
    }

    if (!m_rightEye.addContents({{"left_hand", m_leftHand}, {"right_hand", m_rightHand}, {"l_forearm", m_leftForearm}, {"r_forearm", m_rightForearm}}))
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

    m_leftForearm->set_visibility(m_settings.view_forearms);
    m_rightForearm->set_visibility(m_settings.view_forearms);

    //Initial transforms
    m_leftTransform << 0.0,  0.0,  1.0, 1.0,
                       0.0, -1.0,  0.0, 0.07,
                       1.0,  0.0,  0.0, 0.0,
                       0.0,  0.0,  0.0, 1.0;
    m_leftHand->setTransform(m_leftTransform);

    Eigen::Transform<double, 3, Eigen::Affine> leftForearmInitialTransform(m_leftTransform * m_settings.leftFrameToHand.inverse());
    leftForearmInitialTransform.translate(Eigen::Vector3d({0.0, 0.005, 0.11}));
    leftForearmInitialTransform = leftForearmInitialTransform * m_settings.leftForearmMeshTransform;
    m_leftForearmTrasform->set_transform(leftForearmInitialTransform);

    m_rightTransform << 0.0,  0.0, -1.0,  1.0,
                        0.0,  1.0,  0.0, -0.07,
                        1.0,  0.0,  0.0,  0.0,
                        0.0,  0.0,  0.0,  1.0;
    m_rightHand->setTransform(m_rightTransform);

    Eigen::Transform<double, 3, Eigen::Affine> rightForearmInitialTransform(m_rightTransform * m_settings.rightFrameToHand.inverse());
    rightForearmInitialTransform.translate(Eigen::Vector3d({0.0, 0.005, 0.11}));
    rightForearmInitialTransform = rightForearmInitialTransform * m_settings.rightForearmMeshTransform;
    m_rightForearmTrasform->set_transform(rightForearmInitialTransform);

    m_leftTransformYarp.resize(4,4);
    m_leftTransformYarp.eye();

    m_rightTransformYarp.resize(4,4);
    m_rightTransformYarp.eye();

    m_leftForearmTransformYarp.resize(4,4);
    m_leftForearmTransformYarp.eye();

    m_rightForearmTransformYarp.resize(4,4);
    m_rightForearmTransformYarp.eye();

    std::string rpcPortName = "/" + m_settings.name + "/rpc";
    this->yarp().attachAsServer(this->m_rpcPort);
    if(!m_rpcPort.open(rpcPortName))
    {
        yError() << "Could not open" << rpcPortName << " RPC port.";
        return false;
    }

    return true;
}

bool HandsVisualizer::update()
{
    std::string leftFrame, rightFrame, leftForearmFrame, rightForearmFrame, headFrame;
    bool blocking, viewForearms;
    Eigen::Matrix4d leftFrameToHand, rightFrameToHand, leftForearmMesh, rightForearmMesh;
    {
        std::lock_guard<std::mutex> lock(m_settings.mutex);
        leftFrame = m_settings.left_hand_frame;
        rightFrame = m_settings.right_hand_frame;
        headFrame = m_settings.head_frame;
        leftForearmFrame = m_settings.left_forearm_frame;
        rightForearmFrame = m_settings.right_forearm_frame;
        blocking = m_settings.blocking;
        leftFrameToHand = m_settings.leftFrameToHand;
        rightFrameToHand = m_settings.rightFrameToHand;
        viewForearms = m_settings.view_forearms;
        leftForearmMesh = m_settings.leftForearmMeshTransform;
        rightForearmMesh = m_settings.rightForearmMeshTransform;
    }

    if (m_iframetrans)
    {
        if (m_iframetrans->canTransform(leftFrame, headFrame))
        {
            if (m_iframetrans->getTransform(leftFrame, headFrame, m_leftTransformYarp))
            {
                m_leftTransform = toEigen(m_leftTransformYarp) * leftFrameToHand;
            }
        }

        if (m_iframetrans->canTransform(rightFrame, headFrame))
        {
            if (m_iframetrans->getTransform(rightFrame, headFrame, m_rightTransformYarp))
            {
                m_rightTransform = toEigen(m_rightTransformYarp) * rightFrameToHand;
            }
        }

        if (viewForearms)
        {
            if (m_iframetrans->canTransform(leftForearmFrame, headFrame))
            {
                if (m_iframetrans->getTransform(leftForearmFrame, headFrame, m_leftForearmTransformYarp))
                {
                    m_leftForearmTrasform->set_transform(Eigen::Transform<double, 3, Eigen::Affine>(toEigen(m_leftForearmTransformYarp) * leftForearmMesh));
                }
            }

            if (m_iframetrans->canTransform(rightForearmFrame, headFrame))
            {
                if (m_iframetrans->getTransform(rightForearmFrame, headFrame, m_rightForearmTransformYarp))
                {
                    m_rightForearmTrasform->set_transform(Eigen::Transform<double, 3, Eigen::Affine>(toEigen(m_rightForearmTransformYarp) * rightForearmMesh));
                }
            }
        }
    }

    {
        std::lock_guard<std::mutex> lock(m_handsMutex);
        m_leftHand->setTransform(m_leftTransform);
        m_rightHand->setTransform(m_rightTransform);

        m_leftHand->update(blocking);
        m_rightHand->update(blocking);
        m_leftForearm->update(blocking);
        m_rightForearm->update(blocking);
    }

    {
        std::lock_guard<std::mutex> lock(m_eyesMutex);
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
    }

    m_leftEyeOutputPort.write();
    m_rightEyeOutputPort.write();

    return true;
}

void HandsVisualizer::close()
{
    std::lock_guard<std::mutex> lockSettings(m_settings.mutex), lockEyes(m_eyesMutex), lockHands(m_handsMutex);
    m_rpcPort.close();
    m_leftEyeOutputPort.close();
    m_rightEyeOutputPort.close();
    m_ddtransformclient.close();
}

bool HandsVisualizer::setViewAngle(const double angleInDeg)
{
    std::lock_guard<std::mutex> lockSettings(m_settings.mutex), lockEyes(m_eyesMutex);

    if (!m_leftEye.setViewAngle(angleInDeg) || !m_rightEye.setViewAngle(angleInDeg))
    {
        m_leftEye.setViewAngle(m_settings.viewAngle);
        m_rightEye.setViewAngle(m_settings.viewAngle);
        return false;
    }

    m_settings.viewAngle = angleInDeg;

    return true;

}

bool HandsVisualizer::setHandColor(const double r, const double g, const double b)
{
    std::lock_guard<std::mutex> lockSettings(m_settings.mutex), lockEyes(m_eyesMutex), lockHands(m_handsMutex);
    m_leftHand->setColor({r, g, b});
    m_rightHand->setColor({r, g, b});
    m_leftForearm->set_color({r, g, b});
    m_rightForearm->set_color({r, g, b});
    m_settings.handColor = {r, g, b};
    return true;
}

bool HandsVisualizer::setHandOpacity(const double opacity)
{
    std::lock_guard<std::mutex> lockSettings(m_settings.mutex), lockEyes(m_eyesMutex), lockHands(m_handsMutex);
    m_leftHand->setOpacity(opacity);
    m_rightHand->setOpacity(opacity);
    m_leftForearm->set_opacity(opacity);
    m_rightForearm->set_opacity(opacity);
    m_settings.handOpacity = opacity;
    return true;
}

bool HandsVisualizer::setHeadToLeftEyeOffset(const double x, const double y, const double z)
{
    std::lock_guard<std::mutex> lockSettings(m_settings.mutex), lockEyes(m_eyesMutex);

    if (!m_leftEye.setCameraPosition({x, y, z}))
    {
        return false;
    }

    m_settings.headToLeftEye = {x, y, z};
    return true;
}

bool HandsVisualizer::setHeadToRightEyeOffset(const double x, const double y, const double z)
{
    std::lock_guard<std::mutex> lockSettings(m_settings.mutex), lockEyes(m_eyesMutex);

    if (!m_rightEye.setCameraPosition({x, y, z}))
    {
        return false;
    }

    m_settings.headToRightEye = {x, y, z};
    return true;
}

bool HandsVisualizer::setLeftFrameToHandOffset(const double x, const double y, const double z)
{
    std::lock_guard<std::mutex> lockSettings(m_settings.mutex);

    m_settings.leftFrameToHand.block<3,1>(0, 3) = Eigen::Vector3d({x, y, z});

    return true;

}

bool HandsVisualizer::setRightFrameToHandOffset(const double x, const double y, const double z)
{
    std::lock_guard<std::mutex> lockSettings(m_settings.mutex);

    m_settings.rightFrameToHand.block<3,1>(0, 3) = Eigen::Vector3d({x, y, z});

    return true;
}

bool HandsVisualizer::setLeftFrameToHandQuaternion(const double w, const double x, const double y, const double z)
{
    std::lock_guard<std::mutex> lockSettings(m_settings.mutex);

    m_settings.leftFrameToHand.block<3,3>(0, 0) = Eigen::Quaterniond(w, x, y, z).toRotationMatrix();

    return true;
}

bool HandsVisualizer::setRightFrameToHandQuaternion(const double w, const double x, const double y, const double z)
{
    std::lock_guard<std::mutex> lockSettings(m_settings.mutex);

    m_settings.rightFrameToHand.block<3,3>(0, 0) = Eigen::Quaterniond(w, x, y, z).toRotationMatrix();

    return true;
}

bool HandsVisualizer::setForearmsVisibility(const bool visible)
{
    std::lock_guard<std::mutex> lockSettings(m_settings.mutex), lockEyes(m_eyesMutex), lockHands(m_handsMutex);

    m_leftForearm->set_visibility(visible);
    m_rightForearm->set_visibility(visible);
    m_settings.view_forearms = visible;

    return true;
}

std::string HandsVisualizer::printSettings()
{
    std::string settings;
    {
        std::lock_guard<std::mutex> lockSettings(m_settings.mutex);
        settings = m_settings.toString(0);
    }

    std::vector<std::string> output;
    size_t start = 1;
    size_t newline = settings.find('\n', start);
    while (newline != std::string::npos)
    {
        output.push_back(settings.substr(start, newline - start));
        start = newline + 1;
        newline = settings.find('\n', start);
    }

    //The following is a workaround to print nicely in the RPC terminal
    //all the different strings composing the settings.
    //The code has been copied from the help command.
    yarp::os::idl::WireReader reader(*m_tempReader);
    yarp::os::idl::WireWriter writer(reader);
    if (!writer.isNull()) {
        if (!writer.writeListHeader(2)) {
            return "";
        }
        if (!writer.writeTag("many", 1, 0)) {
            return "";
        }
        if (!writer.writeListBegin(0, output.size() + 1)) { //we give space to an additional string
            return "";
        }
        for (const auto& string : output) {
            if (!writer.writeString(string)) {
                return "";
            }
        }
        m_printSettingsCalled = true; //We still need to close the list. This is done in the read
    }

    return "";
}

bool HandsVisualizer::read(yarp::os::ConnectionReader &connection)
{
    {
        std::lock_guard<std::mutex> lock(m_readMutex); //to protect the write of m_tempReader in case we receive multiple connections
        m_tempReader = &connection;

        if (!HandVisualizerCommands::read(connection))
        {
            return false;
        }
    }

    if (!m_printSettingsCalled)
    {
        return true;
    }
    m_printSettingsCalled = false;

    yarp::os::idl::WireReader reader(connection);
    yarp::os::idl::WireWriter writer(reader);
    if (!writer.writeListEnd()) {
        return false;
    }
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

void HandsVisualizer::Settings::parseAnalogBounds(const yarp::os::Searchable &rf, const std::string &key, yarp::sig::Vector &output, size_t size, double defaultValue)
{
    yarp::sig::Vector defaultVector(size, defaultValue);
    output = defaultVector;
    if (!rf.check(key))
    {
        return;
    }

    yarp::os::Value& value = rf.find(key);

    if (!value.isList())
    {
        yWarning() << key << "found in the configuration parameters, but it is not a list. Using default value.";
        return;
    }

    yarp::os::Bottle* list = value.asList();

    if (list->size() != size)
    {
        yWarning() << key << "found in the configuration parameters, but it is not a list with" << size << "elements. Using default value.";
        return;
    }

    for (size_t i = 0; i < size; ++i)
    {
        if (!list->get(i).isFloat64() && !list->get(i).isInt32() && !list->get(i).isInt64())
        {
            yWarning() << "The element of" << key << "at position" << i << "is not numeric. Using default value.";
        }
        else
        {
            output(i) = list->get(i).asFloat64();
        }
    }
}

void HandsVisualizer::Settings::parse(const yarp::os::Searchable &rf)
{
    robot_name = rf.check("robot", yarp::os::Value("icub")).asString();
    name = rf.check("name", yarp::os::Value("hand-visualizer")).asString();
    blocking = rf.check("blocking") && (rf.find("blocking").isNull() || rf.find("blocking").asBool());
    offscreen = rf.check("offscreen") && (rf.find("offscreen").isNull() || rf.find("offscreen").asBool());
    use_fingers = rf.check("use_fingers") && (rf.find("use_fingers").isNull() || rf.find("use_fingers").asBool());
    use_analogs = rf.check("use_analogs") && (rf.find("use_analogs").isNull() || rf.find("use_analogs").asBool());
    use_analogs_bounds = rf.check("use_analogs_bounds") && (rf.find("use_analogs_bounds").isNull() || rf.find("use_analogs_bounds").asBool());
    filterAbduction = rf.check("filter_abduction") && (rf.find("filter_abduction").isNull() || rf.find("filter_abduction").asBool());
    view_forearms = rf.check("view_forearms") && (rf.find("view_forearms").isNull() || rf.find("view_forearms").asBool());
    head_frame = rf.check("head_frame", yarp::os::Value("head")).asString();
    left_hand_frame = rf.check("left_hand_frame", yarp::os::Value("l_hand")).asString();
    right_hand_frame = rf.check("right_hand_frame", yarp::os::Value("r_hand")).asString();
    left_forearm_frame = rf.check("left_forearm_frame", yarp::os::Value("l_forearm")).asString();
    right_forearm_frame = rf.check("right_forearm_frame", yarp::os::Value("r_forearm")).asString();
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

    //The following offsets have been obtained from the URDF of iCubGazeboV3
    leftForearmMeshTransform.setIdentity();
    rightForearmMeshTransform.setIdentity();
    leftForearmMeshTransform.block<3,1>(0, 3) = parse3DVector(rf, "left_forearm_mesh_offset", {0.03574308828413966, -0.14114024168831188, 0.27323486219617454});
    rightForearmMeshTransform.block<3,1>(0, 3) = parse3DVector(rf, "right_forearm_mesh_offset", {0.03574306124487927, 0.14114026712657127, 0.27323485259309493});
    leftForearmMeshTransform.block<3,3>(0, 0) = parseQuaternion(rf, "left_forearm_mesh_quaternion_wxyz", Eigen::Quaterniond(0.7887826, -0.1163855, -0.0557073, -0.6009768)).matrix();
    rightForearmMeshTransform.block<3,3>(0, 0) = parseQuaternion(rf, "right_forearm_mesh_quaternion_wxyz", Eigen::Quaterniond(0.6009768, 0.0557073, 0.1163854, -0.7887826)).matrix();

    parseAnalogBounds(rf, "left_analog_lower_bounds", left_analog_lower_bounds, 15, 0);
    parseAnalogBounds(rf, "left_analog_upper_bounds", left_analog_upper_bounds, 15, 255);
    parseAnalogBounds(rf, "right_analog_lower_bounds", right_analog_lower_bounds, 15, 0);
    parseAnalogBounds(rf, "right_analog_upper_bounds", right_analog_upper_bounds, 15, 255);
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

    auto yarpVecToString = [](const yarp::sig::Vector& input) -> std::string
    {
        std::stringstream output;
        output << "\"(";
        for (size_t i = 0; i < input.size()-1; ++i)
        {
            output << input(i) << ", ";
        }
        output << *input.cbegin() << ")\"";
        return output.str();
    };

    std::string indentationString(indentation, ' ');


    std::stringstream optionsOut;
    optionsOut << std::endl;
    optionsOut << indentationString << "name " << name << std::endl;
    optionsOut << indentationString << "robot " << robot_name << std::endl;
    optionsOut << indentationString << "blocking " << blocking << std::endl;
    optionsOut << indentationString << "offscreen " << offscreen << std::endl;
    optionsOut << indentationString << "use_fingers " << use_fingers << std::endl;
    optionsOut << indentationString << "use_analogs " << use_analogs << std::endl;
    optionsOut << indentationString << "use_analogs_bounds " << use_analogs_bounds << std::endl;
    optionsOut << indentationString << "left_analog_lower_bounds " << yarpVecToString(left_analog_lower_bounds) << std::endl;
    optionsOut << indentationString << "left_analog_upper_bounds " << yarpVecToString(left_analog_upper_bounds) << std::endl;
    optionsOut << indentationString << "right_analog_lower_bounds " << yarpVecToString(right_analog_lower_bounds) << std::endl;
    optionsOut << indentationString << "right_analog_upper_bounds " << yarpVecToString(right_analog_upper_bounds) << std::endl;
    optionsOut << indentationString << "filter_abduction " << filterAbduction << std::endl;
    optionsOut << indentationString << "view_forearms " << view_forearms << std::endl;
    optionsOut << indentationString << "head_frame " << head_frame << std::endl;
    optionsOut << indentationString << "left_hand_frame " << left_hand_frame << std::endl;
    optionsOut << indentationString << "right_hand_frame " << right_hand_frame << std::endl;
    optionsOut << indentationString << "left_forearm_frame " << left_forearm_frame << std::endl;
    optionsOut << indentationString << "right_forearm_frame " << right_forearm_frame << std::endl;
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
    optionsOut << indentationString << "left_forearm_mesh_offset " << vecToString(leftForearmMeshTransform.block<3,1>(0, 3)) << std::endl;
    optionsOut << indentationString << "right_forearm_mesh_offset " << vecToString(rightForearmMeshTransform.block<3,1>(0, 3)) << std::endl;
    optionsOut << indentationString << "left_forearm_mesh_quaternion_wxyz " << quatToString(Eigen::Quaterniond(leftForearmMeshTransform.block<3,3>(0, 0))) << std::endl;
    optionsOut << indentationString << "right_forearm_mesh_quaternion_wxyz " << quatToString(Eigen::Quaterniond(rightForearmMeshTransform.block<3,3>(0, 0))) << std::endl;
    return optionsOut.str();
}
