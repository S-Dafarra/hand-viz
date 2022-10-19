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

class Eye
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

    std::unique_ptr<VtkContainer> m_vtkContainer{nullptr};
    vtkNew<vtkWindowToImageFilter> m_screenshot;
    yarp::sig::FlexImage m_yarpImage;
    Settings m_settings;

    bool vtkImageDataToYarpimage(vtkImageData* imageData, yarp::sig::FlexImage& image)
    {
        if (!imageData)
        {
            yError() << "[HandVisualizer::vtkImageDataToYarpimage] The input data is not valid.";
            return false;
        }

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
            yError() << "[HandVisualizer::vtkImageDataToYarpimage] Unsopperted number of scalar components.";
            return false;
        }

        image.resize(width, height);

        unsigned char* colorsPtr =
                reinterpret_cast<unsigned char*>(imageData->GetScalarPointer());

        // Loop over the vtkImageData contents.
        for (int row = height - 1; row >= 0; row--) //Flip the image vertically
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

        return true;
    }

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

    bool addContent(const std::string& name, std::shared_ptr<VtkContent> content)
    {
        if (!m_vtkContainer)
        {
            yError() << "[HandVisualizer::addContent] The initialize method had not been called yet.";
            return false;
        }

        if (!content)
        {
            yError() << "[HandVisualizer::addContent] The input content pointer is not valid.";
            return false;
        }

        m_vtkContainer->add_content(name, content);
        return true;
    }

    bool addContents(const std::vector<std::pair<std::string, std::shared_ptr<VtkContent>>>& contents)
    {
        for (auto& content : contents)
        {
            if (!addContent(content.first, content.second))
            {
                return false;
            }
        }
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
    //    m_vtkContainer->render_window()->SetOffScreenRendering(1); //This would be needed to avoid the windows to appear, but it seems not properly supported on Linux
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

    void render()
    {
        if (!m_vtkContainer)
        {
            yError() << "[HandVisualizer::render] The initialize method had not been called yet.";
            return;
        }

        m_vtkContainer->render();
    }

    bool takeScreenshot()
    {
        if (!m_vtkContainer)
        {
            yError() << "[HandVisualizer::takeScreenshot] The initialize method had not been called yet.";
            return false;
        }

        m_screenshot->Modified(); //to update the screenshot
        m_screenshot->Update();
        return vtkImageDataToYarpimage(m_screenshot->GetOutput(), m_yarpImage);
    }

    const yarp::sig::FlexImage& screenshot()
    {
        return m_yarpImage;
    }



};


int main(int argc, char** argv)
{
    //TODO
    //Get the parameters from configuration file
    //RPC
    //Add arms meshes
    //Test the analog readings
    //Cleanup of the code (separate main, reuse code for left and right)
    //Allowing using only one hand and only one camera
    //Test on windows

    yarp::os::Network yarp; //to initialize the network

    std::atomic<bool> closing = false;
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
    leftFrameToHand << 0.0, -1.0,  0.0,  0.015,
                       0.0,  0.0,  1.0, -0.018,
                      -1.0,  0.0,  0.0, -0.055,
                       0.0,  0.0,  0.0,  1.0;
//    leftFrameToHand.block<3,3>(0,0) = Eigen::AngleAxisd(-0.25, Eigen::Vector3d::UnitY()) * leftFrameToHand.block<3,3>(0,0);


    Eigen::Matrix4d rightFrameToHand;
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

    Eye leftEye, rightEye;

    if (!leftEye.initialize(leftSettings))
    {
        return EXIT_FAILURE;
    }

    if (!rightEye.initialize(rightSettings))
    {
        return EXIT_FAILURE;
    }

    /* Show hand according to forward kinematics. */
    std::shared_ptr<VtkiCubHand> leftHand;
    std::shared_ptr<VtkiCubHand> rightHand;
    leftHand = std::make_shared<VtkiCubHand>(robot_name, "left", name + "/hand_fk/left", use_fingers, use_analogs, handColor, handOpacity, useAbduction);
    rightHand = std::make_shared<VtkiCubHand>(robot_name, "right", name + "/hand_fk/right", use_fingers, use_analogs, handColor, handOpacity, useAbduction);

    if (!leftEye.addContents({{"left_hand", leftHand}, {"right_hand", rightHand}}))
    {
        return EXIT_FAILURE;
    }

    if (!rightEye.addContents({{"left_hand", leftHand}, {"right_hand", rightHand}}))
    {
        return EXIT_FAILURE;
    }

    if (!leftEye.prepareVisualization())
    {
        return EXIT_FAILURE;
    }

    if (!rightEye.prepareVisualization())
    {
        return EXIT_FAILURE;
    }


    yarp::os::BufferedPort<yarp::sig::FlexImage> leftEyeOutputPort, rightEyeOutputPort;
    if (!leftEyeOutputPort.open("/" + name + "/left/image"))
    {
        return EXIT_FAILURE;
    }
    if (!rightEyeOutputPort.open("/" + name + "/right/image"))
    {
        return EXIT_FAILURE;
    }


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
    }

    return EXIT_SUCCESS;
}
