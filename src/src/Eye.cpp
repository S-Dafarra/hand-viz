/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <Eye.h>

#include <yarp/os/LogStream.h>


bool Eye::vtkImageDataToYarpimage(vtkImageData *imageData, yarp::sig::FlexImage &image)
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

bool Eye::initialize(const Settings &settings)
{
    if (m_vtkContainer)
    {
        yError() << "[HandVisualizer::initialize] The initialize method had been already called.";
        return false;
    }

    m_settings = settings;
    m_vtkContainer = std::make_unique<RobotsViz::VtkContainer>(1/m_settings.fps, m_settings.width, m_settings.height, m_settings.blocking);

    return true;
}

bool Eye::addContent(const std::string &name, std::shared_ptr<RobotsViz::VtkContent> content)
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

bool Eye::addContents(const std::vector<std::pair<std::string, std::shared_ptr<RobotsViz::VtkContent> > > &contents)
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

bool Eye::setViewAngle(double viewAngle)
{
    if (!m_vtkContainer)
    {
        yError() << "[HandVisualizer::setViewAngle] The initialize method had not been called yet.";
        return false;
    }

    m_settings.viewAngle = viewAngle;

    m_vtkContainer->camera()->SetViewAngle(viewAngle);

    return true;
}

bool Eye::setCameraPosition(const Eigen::Vector3d &position)
{
    if (!m_vtkContainer)
    {
        yError() << "[HandVisualizer::setCameraPosition] The initialize method had not been called yet.";
        return false;
    }

    m_settings.cameraPosition = position;

    m_vtkContainer->camera()->SetPosition(m_settings.cameraPosition(0), m_settings.cameraPosition(1), m_settings.cameraPosition(2));

    return true;
}

bool Eye::prepareVisualization()
{
    if (!m_vtkContainer)
    {
        yError() << "[HandVisualizer::prepareVisualization] The initialize method had not been called yet.";
        return false;
    }

    m_vtkContainer->initialize();
    m_vtkContainer->setOrientationWidgetEnabled(false);
    m_vtkContainer->render_window()->SetAlphaBitPlanes(1);
    if (m_settings.offscreen)
    {
        m_vtkContainer->render_window()->SetOffScreenRendering(1);
    }
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

void Eye::render()
{
    if (!m_vtkContainer)
    {
        yError() << "[HandVisualizer::render] The initialize method had not been called yet.";
        return;
    }

    m_vtkContainer->render();
}

bool Eye::takeScreenshot()
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

const yarp::sig::FlexImage &Eye::screenshot()
{
    return m_yarpImage;
}
