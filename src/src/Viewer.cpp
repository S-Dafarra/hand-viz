/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Camera/Camera.h>
#include <RobotsIO/Camera/iCubCamera.h>

#include <RobotsViz/VtkContainer.h>
#include <RobotsViz/VtkContent.h>
#include <RobotsViz/VtkPointCloud.h>
#include <RobotsViz/VtkiCubHand.h>

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>

#include <vtkWindowToImageFilter.h>
#include <vtkNew.h>

#include <functional>
#include <cstring>
#include <thread>
#include <csignal>
#include <yarp/os/LogStream.h>
#include <yarp/sig/Image.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IFrameTransform.h>

using namespace RobotsIO::Camera;
using namespace RobotsViz;

std::function<void ()> customHandlerLambda;

void my_handler(int sig)
{
    static int ct = 0;

    if (sig == SIGABRT)
    {
        yInfo() << "Aborted.";
        if (ct > 3) //to avoid that std::abort is called again
        {
            return;
        }
    }

    ct++;
    if (ct > 3) {
        yInfo() <<  "Aborting (calling abort())...";
        std::abort();
    }
    yInfo() << "[try " << ct << " of 3] Trying to shut down.";

    customHandlerLambda();
}

#ifdef WIN32

#include <windows.h>

BOOL WINAPI CtrlHandler(DWORD fdwCtrlType)
{
    switch (fdwCtrlType) {
        // Handle the CTRL-C signal.
    case CTRL_C_EVENT:
    case CTRL_CLOSE_EVENT:
    case CTRL_SHUTDOWN_EVENT:
        my_handler(0);
        return TRUE;

    // Handle all other events
    default:
        return FALSE;
    }
}
#endif

void handleSignals(std::function<void ()> customHandler)
{
#ifdef WIN32
    SetConsoleCtrlHandler(CtrlHandler, TRUE);
#else
    struct sigaction action;
    memset(&action, 0, sizeof(action));
    action.sa_handler = &my_handler;
    sigaction(SIGINT, &action, NULL);
    sigaction(SIGTERM, &action, NULL);
    sigaction(SIGABRT, &action, NULL);
#endif
    customHandlerLambda = customHandler;
}

void vtkImageDataToYarpimage(vtkImageData* imageData, yarp::sig::FlexImage& image)
{
    if (!imageData)
    {
        return;
    }

    /// \todo retrieve just the UpdateExtent
    int width = imageData->GetDimensions()[0];
    int height = imageData->GetDimensions()[1];

    bool useRGBA = false;
    if (imageData->GetNumberOfScalarComponents() == 3)
    {
        image.setPixelCode(VOCAB_PIXEL_RGB);
    }
    else if (imageData->GetNumberOfScalarComponents() > 3)
    {
        image.setPixelCode(VOCAB_PIXEL_RGBA);
        useRGBA = true;
    }
    else
    {
        return;
    }

    image.resize(width, height);

    unsigned char* colorsPtr =
            reinterpret_cast<unsigned char*>(imageData->GetScalarPointer());

    // Loop over the vtkImageData contents.
    for (int row = height - 1; row >= 0; row--)
    {
        for (int col = 0; col < width; col++)
        {
            if (useRGBA)
            {
                yarp::sig::PixelRgba& pixelYarp = *(reinterpret_cast<yarp::sig::PixelRgba*>(
                                                        image.getPixelAddress(col, row)));

                //iDynTree specifies the pixels in [0.0, 1.0], yarp between in [0, 255]
                pixelYarp.r = colorsPtr[0];
                pixelYarp.g = colorsPtr[1];
                pixelYarp.b = colorsPtr[2];
                pixelYarp.a = colorsPtr[3];
            }
            else
            {
                yarp::sig::PixelRgb& pixelYarp = *(reinterpret_cast<yarp::sig::PixelRgb*>(
                                                       image.getPixelAddress(col, row)));

                //iDynTree specifies the pixels in [0.0, 1.0], yarp between in [0, 255]
                pixelYarp.r = colorsPtr[0];
                pixelYarp.g = colorsPtr[1];
                pixelYarp.b = colorsPtr[2];
            }
            // Swap the vtkImageData RGB values with an equivalent QColor
            colorsPtr += imageData->GetNumberOfScalarComponents();
        }
    }
}

Eigen::Matrix4d toEigen(const yarp::sig::Matrix& input)
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

class HandVisualizer
{
public:

    struct Settings
    {
        double fps{30};
        unsigned int width{600};
        unsigned int height{600};
        bool blocking{false};
        std::tuple<double, double, double> backgroundColor{0,0,0};
        Eigen::Vector3d cameraPosition = Eigen::Vector3d::Zero();
        Eigen::Vector3d forwardDirection = Eigen::Vector3d::UnitX();
        Eigen::Vector3d upDirection = Eigen::Vector3d::UnitZ();
        double viewAngle{85};
    };

private:

    std::unordered_map<std::string, std::shared_ptr<VtkiCubHand>> m_hands;
    std::unique_ptr<VtkContainer> m_vtkContainer{nullptr};
    vtkNew<vtkWindowToImageFilter> m_screenshot;
    yarp::sig::FlexImage m_yarpImage;
    Settings m_settings;

public:



    bool initialize(const Settings& settings)
    {
        if (m_vtkContainer)
        {
            yError() << "[HandVisualizer::initialize] The initialize method had been already called.";
            return false;
        }

        m_settings = settings;
        m_vtkContainer = std::make_unique<VtkContainer>(1/m_settings.fps, m_settings.width, m_settings.height, m_settings.blocking);

        return true;
    }

    bool addHand(const std::string& name, std::shared_ptr<VtkiCubHand> hand)
    {
        if (!m_vtkContainer)
        {
            yError() << "[HandVisualizer::addHand] The initialize method had not been called yet.";
            return false;
        }

        if (m_hands.find(name) != m_hands.end())
        {
            yError() << "[HandVisualizer::addHand] The hand named" << name << "already exists";
            return false;
        }

        if (!hand)
        {
            yError() << "[HandVisualizer::addHand] The input hand pointer is not valid.";
            return false;
        }

        m_hands[name] = hand;
        m_vtkContainer->add_content(name, hand);
        return true;
    }

    bool prepareVisualization()
    {
        if (!m_vtkContainer)
        {
            yError() << "[HandVisualizer::prepareVisualization] The initialize method had not been called yet.";
            return false;
        }

        m_vtkContainer->initialize();
        m_vtkContainer->setOrientationWidgetEnabled(false);
        m_vtkContainer->render_window()->SetAlphaBitPlanes(1);
    //    leftEyeContainer.render_window()->SetOffScreenRendering(1); //Not properly supported on Linux
        m_vtkContainer->renderer()->SetBackground(std::get<0>(m_settings.backgroundColor),
                                                  std::get<1>(m_settings.backgroundColor),
                                                  std::get<2>(m_settings.backgroundColor));
        m_vtkContainer->camera()->SetViewUp(m_settings.upDirection(0), m_settings.upDirection(1), m_settings.upDirection(2));
        m_vtkContainer->camera()->SetPosition(m_settings.cameraPosition(0), m_settings.cameraPosition(1), m_settings.cameraPosition(2));
        m_vtkContainer->camera()->SetFocalPoint(m_settings.cameraPosition(0) + m_settings.forwardDirection(0),
                                                m_settings.cameraPosition(1) + m_settings.forwardDirection(1),
                                                m_settings.cameraPosition(2) + m_settings.forwardDirection(2));
        m_vtkContainer->camera()->SetViewAngle(m_settings.viewAngle);
        m_screenshot->SetInputBufferTypeToRGBA();
        m_screenshot->SetInput(m_vtkContainer->render_window());

        return true;
    }



};


int main(int argc, char** argv)
{
    //TODO
    //Get the parameters from configuration file
    //RPC
    //Test the analog readings
    //Cleanup of the code (separate main, reuse code for left and right)
    //Allowing using only one hand and only one camera
    //Test on windows

    yarp::os::Network yarp; //to initialize the network

    bool closing = false;
    handleSignals([&](){closing = true;});

    if (argc < 2)
    {
        std::cout << "Synopsis: icub-hand-viz <robot_name>" << std::endl;
        return EXIT_FAILURE;
    }

    //------------Parameters---------------
    const std::string robot_name = std::string(argv[1]);

    std::string name = "hand-visualizer";

    bool blocking = false;
    bool use_fingers = true;
    bool use_analogs = false;
    std::string head_frame = "head";
    std::string left_frame = "l_hand";
    std::string right_frame = "r_hand";

    Eigen::Vector3d headToLeftEye;
    headToLeftEye << 0.051, 0.034, 0.013;

    Eigen::Vector3d headToRightEye;
    headToRightEye <<  0.051, -0.034, 0.013;

    Eigen::Matrix4d leftFrameToHand;
//    leftFrameToHand << 0.0, -1.0,  0.0, -0.002,
//                       0.0,  0.0,  1.0, -0.018,
//                      -1.0,  0.0,  0.0, -0.059,
//                       0.0,  0.0,  0.0,  1.0;
    leftFrameToHand << 0.0, -1.0,  0.0,  0.025,
                       0.0,  0.0,  1.0, -0.018,
                      -1.0,  0.0,  0.0, -0.055,
                       0.0,  0.0,  0.0,  1.0;
    leftFrameToHand.block<3,3>(0,0) = Eigen::AngleAxisd(-0.25, Eigen::Vector3d::UnitY()) * leftFrameToHand.block<3,3>(0,0);


    Eigen::Matrix4d rightFrameToHand;
//    rightFrameToHand << 0.0, -1.0,  0.0, -0.002,
//                        0.0,  0.0,  1.0,  0.018,
//                       -1.0,  0.0,  0.0, -0.059,
//                        0.0,  0.0,  0.0,  1.0;
    rightFrameToHand << 0.0, -1.0,  0.0,  0.010,
                        0.0,  0.0,  1.0,  0.010,
                       -1.0,  0.0,  0.0, -0.050,
                        0.0,  0.0,  0.0,  1.0;
    rightFrameToHand.block<3,3>(0,0) = Eigen::AngleAxisd(-0.25, Eigen::Vector3d::UnitY()) * rightFrameToHand.block<3,3>(0,0);



    double viewAngle = 37;

    double fps = 30.0;

    Eigen::Vector3d forwardDirection = Eigen::Vector3d::UnitX();
    Eigen::Vector3d upDirection = Eigen::Vector3d::UnitZ();


    std::tuple<double, double, double> handColor{100.0 / 255.0, 160 / 255.0, 255.0 / 255.0};
    double handOpacity = 0.2;

    bool useAbduction = false;
    int windowWidth = 320;
    int windowHeight = 240;

    std::string tfRemote = "/transformServer";

    //------------------------------------------


    yarp::dev::PolyDriver       ddtransformclient;
    yarp::dev::IFrameTransform       *iframetrans{nullptr};

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

    VtkContainer leftEyeContainer(1.0 / fps, windowWidth, windowHeight, blocking);
    VtkContainer rightEyeContainer(1.0 / fps, windowWidth, windowHeight, blocking);


    /* Show hand according to forward kinematics. */
    std::shared_ptr<VtkiCubHand> leftHand;
    std::shared_ptr<VtkiCubHand> rightHand;
    leftHand = std::make_shared<VtkiCubHand>(robot_name, "left", name + "/hand_fk/left", use_fingers, use_analogs, handColor, handOpacity, useAbduction);
    rightHand = std::make_shared<VtkiCubHand>(robot_name, "right", name + "/hand_fk/right", use_fingers, use_analogs, handColor, handOpacity, useAbduction);
    leftEyeContainer.add_content("hand_fk_l", leftHand);
    rightEyeContainer.add_content("hand_fk_l", leftHand);
    leftEyeContainer.add_content("hand_fk_r", rightHand);
    rightEyeContainer.add_content("hand_fk_r", rightHand);

    yarp::sig::FlexImage yarpImage;
    yarp::os::BufferedPort<yarp::sig::FlexImage> leftEyeOutputPort, rightEyeOutputPort;
    if (!leftEyeOutputPort.open("/" + name + "/left/image"))
    {
        return EXIT_FAILURE;
    }
    if (!rightEyeOutputPort.open("/" + name + "/right/image"))
    {
        return EXIT_FAILURE;
    }


    leftEyeContainer.initialize();
    leftEyeContainer.setOrientationWidgetEnabled(false);
    leftEyeContainer.render_window()->SetAlphaBitPlanes(1);
//    leftEyeContainer.render_window()->SetOffScreenRendering(1); //Not properly supported on Linux
    leftEyeContainer.renderer()->SetBackground(0, 0, 0);
    leftEyeContainer.camera()->SetViewUp(upDirection(0), upDirection(1), upDirection(2));
    leftEyeContainer.camera()->SetPosition(headToLeftEye(0), headToLeftEye(1), headToLeftEye(2));
    leftEyeContainer.camera()->SetFocalPoint(headToLeftEye(0) + forwardDirection(0),
                                             headToLeftEye(1) + forwardDirection(1),
                                             headToLeftEye(2) + forwardDirection(2));
    leftEyeContainer.camera()->SetViewAngle(viewAngle);


    rightEyeContainer.initialize();
    rightEyeContainer.setOrientationWidgetEnabled(false);
    rightEyeContainer.render_window()->SetAlphaBitPlanes(1);
//    rightEyeContainer.render_window()->SetOffScreenRendering(1); //Not properly supported on Linux
    rightEyeContainer.renderer()->SetBackground(0, 0, 0);
    rightEyeContainer.camera()->SetViewUp(upDirection(0), upDirection(1), upDirection(2));
    rightEyeContainer.camera()->SetPosition(headToRightEye(0), headToRightEye(1), headToRightEye(2));
    rightEyeContainer.camera()->SetFocalPoint(headToRightEye(0) + forwardDirection(0),
                                              headToRightEye(1) + forwardDirection(1),
                                              headToRightEye(2) + forwardDirection(2));
    rightEyeContainer.camera()->SetViewAngle(viewAngle);


    vtkNew<vtkWindowToImageFilter> rightScreenshot;
    rightScreenshot->SetInputBufferTypeToRGBA();
    rightScreenshot->SetInput(rightEyeContainer.render_window());
    vtkNew<vtkWindowToImageFilter> leftScreenshot;
    leftScreenshot->SetInputBufferTypeToRGBA();
    leftScreenshot->SetInput(leftEyeContainer.render_window());

    Eigen::Matrix4d leftTransform;
    leftTransform << 0.0,  0.0,  1.0, 1.0,
                     0.0, -1.0,  0.0, 0.05,
                     1.0,  0.0,  0.0, 0.0,
                     0.0,  0.0,  0.0, 1.0;

    leftHand->setTransform(leftTransform);

    Eigen::Matrix4d rightTransform;
    rightTransform << 0.0,  0.0, -1.0,  1.0,
                      0.0,  1.0,  0.0, -0.05,
                      1.0,  0.0,  0.0,  0.0,
                      0.0,  0.0,  0.0,  1.0;
    rightHand->setTransform(rightTransform);


    yarp::sig::Matrix leftTransformYarp, rightTransformYarp;

    leftTransformYarp.resize(4,4);
    leftTransformYarp.eye();

    rightTransformYarp.resize(4,4);
    rightTransformYarp.eye();


    while(!closing)
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


        leftEyeContainer.updateContent(); //the content is shared between the two

        leftEyeContainer.render();
        rightEyeContainer.render();

        leftScreenshot->Modified(); //to update the screenshot
        leftScreenshot->Update();
        vtkImageDataToYarpimage(leftScreenshot->GetOutput(), yarpImage);

        yarp::sig::FlexImage& imageLeftToBeSent = leftEyeOutputPort.prepare();
        imageLeftToBeSent.setPixelCode(yarpImage.getPixelCode());
        imageLeftToBeSent.setExternal(yarpImage.getRawImage(), yarpImage.width(), yarpImage.height()); //Avoid to copy
        leftEyeOutputPort.write();

        rightScreenshot->Modified(); //to update the screenshot
        rightScreenshot->Update();
        vtkImageDataToYarpimage(rightScreenshot->GetOutput(), yarpImage);

        yarp::sig::FlexImage& imageRightToBeSent = rightEyeOutputPort.prepare();
        imageRightToBeSent.setPixelCode(yarpImage.getPixelCode());
        imageRightToBeSent.setExternal(yarpImage.getRawImage(), yarpImage.width(), yarpImage.height()); //Avoid to copy
        rightEyeOutputPort.write();
    }

    return EXIT_SUCCESS;
}
