#ifndef KINECTFUSION_CAMERA_H
#define KINECTFUSION_CAMERA_H

/*
 * Camera class declarations. Add your own camera handler by deriving from DepthCamera.
 * Author: Christian Diller
 */

#include <data_types.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Wextra"
#pragma GCC diagnostic ignored "-Weffc++"
#pragma GCC diagnostic ignored "-Wunused-variable"
#include <OpenNI.h>
#pragma GCC diagnostic pop

#include <librealsense2/rs.hpp>

using kinectfusion::CameraParameters;

/**
 * Represents a single input frame
 * Packages a depth map with the corresponding RGB color map
 * The depth map is expected to hold float values, the color map 8 bit RGB values
 */
struct InputFrame {
    cv::Mat_<float> depth_map;
    cv::Mat_<cv::Vec3b> color_map;
};

/*
 * Models the interface to a device that provides raw depth images
 */
class DepthCamera {
public:
    virtual ~DepthCamera() = default;

    virtual InputFrame grab_frame() = 0;
    virtual CameraParameters get_parameters() const = 0;
};

/*
 * For testing purposes. This camera simply loads depth frames stored on disk.
 */
class PseudoCamera : public DepthCamera {
public:
    explicit PseudoCamera(const std::string& _data_path);
    ~PseudoCamera() override = default;

    InputFrame grab_frame() override;
    CameraParameters get_parameters() const override;

private:
    std::string data_path;
    CameraParameters cam_params;
    mutable size_t current_index;
};

/*
 * Provides depth frames acquired by a Asus Xtion PRO LIVE camera.
 */
class XtionCamera : public DepthCamera {
public:
    XtionCamera();
    ~XtionCamera() override = default;

    InputFrame grab_frame() override;

    CameraParameters get_parameters() const override;

private:
    openni::Device device;
    mutable openni::VideoStream depthStream;
    mutable openni::VideoStream colorStream;
    mutable openni::VideoFrameRef depthFrame;
    mutable openni::VideoFrameRef colorFrame;

    CameraParameters cam_params;
};

/*
 * Provides depth frames acquired by an Intel Realsense camera.
 */
class RealSenseCamera : public DepthCamera {
public:
    RealSenseCamera();
    RealSenseCamera(const std::string& filename);

    ~RealSenseCamera() override;

    InputFrame grab_frame() override;

    CameraParameters get_parameters() const override;

private:

    std::vector<std::vector<float>> makeIDWKernel(
        const int& size
    ) const;

    uint16_t get_IDW_value(
        const int& x, const int& y,
        const uint16_t* out_z,
        const int& src_width, const int& src_height
    ) const;

    void IDW_hole_fill(cv::Mat& depth_map);

    rs2::pipeline pipeline;
    rs2::align aligner;

    CameraParameters cam_params;

    float depth_scale;

    uint16_t* working_buf = nullptr;
    std::vector<std::vector<float>> idw_kernel;
};


/*
 * Provides depth frames acquired by a Microsoft Kinect camera.
 */
/*
class KinectCamera : public DepthCamera {
public:
    KinectCamera();

    ~KinectCamera();

    InputFrame grab_frame() const override;

    CameraParameters get_parameters() const override;
};
 */

#endif //KINECTFUSION_CAMERA_H
