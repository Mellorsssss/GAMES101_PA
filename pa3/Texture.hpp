//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture
{
private:
    cv::Mat image_data;

    static inline void prepare(float &u, float &v)
    {
        if (u < 0)
            u = 0;
        else if (u > 1)
            u = 1;
        if (v < 0)
            v = 0;
        else if (v > 1)
            v = 1;
    }

    static inline void lerp(float &s, const float &mid, const float &begin, const float &end)
    {
        s = (mid - begin) / (end - begin);
    }

public:
    Texture(const std::string &name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        prepare(u, v);
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        prepare(u, v);
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        float lb_x = floor(u_img);
        float lb_y = floor(v_img);
        float rt_x = std::min((float)width, ceil(u_img));
        float rt_y = std::min((float)height, ceil(v_img));

        auto color_lb = image_data.at<cv::Vec3b>(lb_y, lb_x);
        auto color_lt = image_data.at<cv::Vec3b>(rt_y, lb_x);
        auto color_rb = image_data.at<cv::Vec3b>(lb_y, rt_x);
        auto color_rt = image_data.at<cv::Vec3b>(rt_y, rt_x);

        float s, t;
        lerp(s, u_img, lb_x, rt_x);
        lerp(t, v_img, lb_y, rt_y);
        auto color_sb = (1 - s) * color_lb + s * color_rb;
        auto color_st = (1 - s) * color_lt + s * color_rt;
        auto color_tt = (1 - t) * color_sb + t * color_st;
        return Eigen::Vector3f(color_tt[0], color_tt[1], color_tt[2]);
    }
};
#endif //RASTERIZER_TEXTURE_H
