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

Eigen::Vector3d HandsVisualizer::parse3DVector(const yarp::os::ResourceFinder &rf, const std::string &key, const Eigen::Vector3d &defaultValue)
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

Eigen::Quaterniond HandsVisualizer::parseQuaternion(const yarp::os::ResourceFinder &rf, const std::string &key, const Eigen::Quaterniond &defaultValue)
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

bool HandsVisualizer::configure(const yarp::os::ResourceFinder &rf)
{
    //------------Parameters---------------
    std::string robot_name = rf.check("robot", yarp::os::Value("icub")).asString();
    std::string name = rf.check("name", yarp::os::Value("hand-visualizer")).asString();
    blocking = rf.check("blocking") && (rf.find("blocking").isNull() || rf.find("blocking").asBool());
    bool use_fingers = rf.check("use_fingers") && (rf.find("use_fingers").isNull() || rf.find("use_fingers").asBool());
    bool use_analogs = rf.check("use_analogs") && (rf.find("use_analogs").isNull() || rf.find("use_analogs").asBool());
    bool filterAbduction = rf.check("filter_abduction") && (rf.find("filter_abduction").isNull() || rf.find("filter_abduction").asBool());
    head_frame = rf.check("head_frame", yarp::os::Value("head")).asString();
    left_frame = rf.check("left_frame", yarp::os::Value("l_hand")).asString();
    right_frame = rf.check("right_frame", yarp::os::Value("r_hand")).asString();
    double viewAngle = rf.check("view_angle", yarp::os::Value(85.0)).asFloat64();
    double fps = rf.check("desired_fps", yarp::os::Value(30.0)).asFloat64();
    double handOpacity = rf.check("hand_opacity", yarp::os::Value(1.0)).asFloat64();
    Eigen::Vector3d handColor= parse3DVector(rf, "hand_color", {100.0 / 255.0, 160 / 255.0, 255.0 / 255.0});
    int windowWidth = rf.check("window_width", yarp::os::Value(600)).asInt32();
    int windowHeight = rf.check("window_height", yarp::os::Value(600)).asInt32();
    std::string tfRemote = rf.check("tf_remote", yarp::os::Value("/transformServer")).asString();
    Eigen::Vector3d headToLeftEye = parse3DVector(rf, "head_to_left_eye_offset", {0.051, 0.034, 0.013});
    Eigen::Vector3d headToRightEye = parse3DVector(rf, "head_to_right_eye_offset", {0.051, -0.034, 0.013});
    Eigen::Vector3d forwardDirection = parse3DVector(rf, "forward_direction", Eigen::Vector3d::UnitX());
    Eigen::Vector3d upDirection = parse3DVector(rf, "up_direction", Eigen::Vector3d::UnitZ());

    leftFrameToHand.setIdentity();
    rightFrameToHand.setIdentity();
    leftFrameToHand.block<3,1>(0, 3) = parse3DVector(rf, "left_frame_to_hand_offset", {-0.002, -0.018, -0.059});
    rightFrameToHand.block<3,1>(0, 3) = parse3DVector(rf, "right_frame_to_hand_offset", {-0.002, 0.018, -0.059});
    leftFrameToHand.block<3,3>(0, 0) = parseQuaternion(rf, "left_frame_to_hand_quaternion_wxyz", Eigen::Quaterniond(-0.5, 0.5, -0.5, -0.5)).matrix();
    rightFrameToHand.block<3,3>(0, 0) = parseQuaternion(rf, "right_frame_to_hand_quaternion_wxyz", Eigen::Quaterniond(-0.5, 0.5, -0.5, -0.5)).matrix();

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


    std::stringstream optionsOut;
    optionsOut << "Using the following settings:" << std::endl;
    optionsOut << "        name " << name << std::endl;
    optionsOut << "        robot " << robot_name << std::endl;
    optionsOut << "        blocking " << blocking << std::endl;
    optionsOut << "        use_fingers " << use_fingers << std::endl;
    optionsOut << "        use_analogs " << use_analogs << std::endl;
    optionsOut << "        filter_abduction " << filterAbduction << std::endl;
    optionsOut << "        head_frame " << head_frame << std::endl;
    optionsOut << "        left_frame " << left_frame << std::endl;
    optionsOut << "        right_frame " << right_frame << std::endl;
    optionsOut << "        view_angle " << viewAngle << std::endl;
    optionsOut << "        desired_fps " << fps << std::endl;
    optionsOut << "        hand_opacity " << handOpacity << std::endl;
    optionsOut << "        hand_color " << vecToString(handColor) << std::endl;
    optionsOut << "        window_width " << windowWidth << std::endl;
    optionsOut << "        window_height " << windowHeight << std::endl;
    optionsOut << "        tf_remote " << tfRemote << std::endl;
    optionsOut << "        head_to_left_eye_offset " << vecToString(headToLeftEye) << std::endl;
    optionsOut << "        head_to_right_eye_offset " << vecToString(headToRightEye) << std::endl;
    optionsOut << "        forward_direction " << vecToString(forwardDirection) << std::endl;
    optionsOut << "        up_direction " << vecToString(upDirection) << std::endl;
    optionsOut << "        left_frame_to_hand_offset " << vecToString(leftFrameToHand.block<3,1>(0, 3)) << std::endl;
    optionsOut << "        right_frame_to_hand_offset " << vecToString(rightFrameToHand.block<3,1>(0, 3)) << std::endl;
    optionsOut << "        left_frame_to_hand_quaternion_wxyz " << quatToString(Eigen::Quaterniond(leftFrameToHand.block<3,3>(0, 0))) << std::endl;
    optionsOut << "        right_frame_to_hand_quaternion_wxyz " << quatToString(Eigen::Quaterniond(rightFrameToHand.block<3,3>(0, 0))) << std::endl;
    yInfo() << optionsOut.str();


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

    leftHand = std::make_shared<RobotsViz::VtkiCubHand>(robot_name, "left", name + "/hand_fk/left", use_fingers, use_analogs,
                                                        std::tuple<double, double, double>({handColor(0), handColor(1),handColor(2)}), handOpacity, !filterAbduction);
    rightHand = std::make_shared<RobotsViz::VtkiCubHand>(robot_name, "right", name + "/hand_fk/right", use_fingers, use_analogs,
                                                         std::tuple<double, double, double>({handColor(0), handColor(1),handColor(2)}), handOpacity, !filterAbduction);

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
