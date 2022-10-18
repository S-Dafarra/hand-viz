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


int main(int argc, char** argv)
{
    yarp::os::Network yarp; //to initialize the network

    bool closing = false;
    handleSignals([&](){closing = true;});

    if (argc < 2)
    {
        std::cout << "Synopsis: icub-hand-viz <robot_name>" << std::endl;
        return EXIT_FAILURE;
    }
    const std::string robot_name = std::string(argv[1]);

    bool blocking = false;
    bool use_fingers = true;
    bool use_analogs = false;
    std::string head_frame = "head";
    std::string left_frame = "l_hand";
    std::string right_frame = "r_hand";

    Eigen::Matrix4d headToLeftEye;
    headToLeftEye << 1.0,  0.0,  0.0,  0.051,
                     0.0,  1.0,  0.0,  0.034,
                     0.0,  0.0,  1.0,  0.013,
                     0.0,  0.0,  0.0,    1.0;



    Eigen::Matrix4d headToRightEye;
    headToRightEye << 1.0,  0.0,  0.0,  0.051,
                      0.0,  1.0,  0.0, -0.034,
                      0.0,  0.0,  1.0,  0.013,
                      0.0,  0.0,  0.0,    1.0;

    Eigen::Matrix4d leftFrameToHand;
    leftFrameToHand << 0.0, -1.0,  0.0, -0.002,
                       0.0,  0.0,  1.0, -0.018,
                      -1.0,  0.0,  0.0, -0.059,
                       0.0,  0.0,  0.0,  1.0;


    Eigen::Matrix4d rightFrameToHand;
    rightFrameToHand << 0.0, -1.0,  0.0, -0.002,
                        0.0,  0.0,  1.0,  0.018,
                       -1.0,  0.0,  0.0, -0.059,
                        0.0,  0.0,  0.0,  1.0;


    double fps = 30.0;



    yarp::dev::PolyDriver       ddtransformclient;
    yarp::dev::IFrameTransform       *iframetrans{nullptr};

    yarp::os::Property pTransformclient_cfg;
    pTransformclient_cfg.put("device", "transformClient");
    pTransformclient_cfg.put("local", "/hand-visualizer/transformClient");
    pTransformclient_cfg.put("remote",  "/transformServer");

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

    VtkContainer leftEyeContainer(1.0 / fps, 600, 600, blocking);
    VtkContainer rightEyeContainer(1.0 / fps, 600, 600, blocking);


    /* Show hand according to forward kinematics. */
    std::shared_ptr<VtkiCubHand> leftHand;
    std::shared_ptr<VtkiCubHand> rightHand;
    leftHand = std::make_shared<VtkiCubHand>(robot_name, "left", "test-visualization/hand_fk/left", use_fingers, use_analogs, std::tuple<double, double, double>{100.0 / 255.0, 160 / 255.0, 255.0 / 255.0}, 1.0, false);
    rightHand = std::make_shared<VtkiCubHand>(robot_name, "right", "test-visualization/hand_fk/right", use_fingers, use_analogs, std::tuple<double, double, double>{100.0 / 255.0, 160 / 255.0, 255.0 / 255.0}, 1.0, false);
    leftEyeContainer.add_content("hand_fk_l", leftHand);
    rightEyeContainer.add_content("hand_fk_l", leftHand);
    leftEyeContainer.add_content("hand_fk_r", rightHand);
    rightEyeContainer.add_content("hand_fk_r", rightHand);

    yarp::sig::FlexImage yarpImage;
    yarp::os::BufferedPort<yarp::sig::FlexImage> leftEyeOutputPort, rightEyeOutputPort;
    if (!leftEyeOutputPort.open("/hand-visualizer/left/image"))
    {
        return EXIT_FAILURE;
    }
    if (!rightEyeOutputPort.open("/hand-visualizer/right/image"))
    {
        return EXIT_FAILURE;
    }


    leftEyeContainer.initialize();
    leftEyeContainer.setOrientationWidgetEnabled(false);
    leftEyeContainer.render_window()->SetAlphaBitPlanes(1);
//    leftEyeContainer.render_window()->SetOffScreenRendering(1); //Not properly supported on Linux
    leftEyeContainer.renderer()->SetBackground(0, 0, 0);
    leftEyeContainer.camera()->SetViewUp(0.0, 0.0, 1.0);
    leftEyeContainer.camera()->SetPosition(headToLeftEye(0, 3) - 1.0, headToLeftEye(1, 3), headToLeftEye(2, 3));
    leftEyeContainer.camera()->SetFocalPoint(headToLeftEye(0, 3), headToLeftEye(1, 3), headToLeftEye(2, 3));


    rightEyeContainer.initialize();
    rightEyeContainer.setOrientationWidgetEnabled(false);
    rightEyeContainer.render_window()->SetAlphaBitPlanes(1);
//    rightEyeContainer.render_window()->SetOffScreenRendering(1); //Not properly supported on Linux
    rightEyeContainer.renderer()->SetBackground(0, 0, 0);
    rightEyeContainer.camera()->SetViewUp(0.0, 0.0, 1.0);
    rightEyeContainer.camera()->SetPosition(headToRightEye(0, 3) - 1.0, headToRightEye(1, 3), headToRightEye(2, 3));
    rightEyeContainer.camera()->SetFocalPoint(headToRightEye(0, 3), headToRightEye(1, 3), headToRightEye(2, 3));


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
