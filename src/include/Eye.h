/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef HAND_VIZ_EYE_H
#define HAND_VIZ_EYE_H

#include <RobotsViz/VtkContainer.h>
#include <RobotsViz/VtkiCubHand.h>

#include <vtkWindowToImageFilter.h>
#include <vtkNew.h>

#include <yarp/sig/Image.h>

#include <Eigen/Dense>
#include <tuple>
#include <memory>

class Eye
{
public:

    struct Settings
    {
        double fps{30};
        unsigned int width{600};
        unsigned int height{600};
        bool blocking{false};
        bool offscreen{false};
        std::tuple<double, double, double> backgroundColor{0,0,0};
        Eigen::Vector3d cameraPosition = Eigen::Vector3d::Zero();
        Eigen::Vector3d forwardDirection = Eigen::Vector3d::UnitX();
        Eigen::Vector3d upDirection = Eigen::Vector3d::UnitZ();
        double viewAngle{85};
    };

    bool initialize(const Settings& settings);

    bool addContent(const std::string& name, std::shared_ptr<RobotsViz::VtkContent> content);

    bool addContents(const std::vector<std::pair<std::string, std::shared_ptr<RobotsViz::VtkContent>>>& contents);

    bool setViewAngle(double viewAngle);

    bool setCameraPosition(const Eigen::Vector3d& position);

    bool prepareVisualization();

    void render();

    bool takeScreenshot();

    const yarp::sig::FlexImage& screenshot();

private:

    std::unique_ptr<RobotsViz::VtkContainer> m_vtkContainer{nullptr};
    vtkNew<vtkWindowToImageFilter> m_screenshot;
    yarp::sig::FlexImage m_yarpImage;
    Settings m_settings;

    bool vtkImageDataToYarpimage(vtkImageData* imageData, yarp::sig::FlexImage& image);

};

#endif // HAND_VIZ_EYE_H
