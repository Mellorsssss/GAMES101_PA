#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

inline float cal_radian(float angle){
    return angle*MY_PI/180.0;
}

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 
                 0, 1, 0, -eye_pos[1], 
                 0, 0, 1,-eye_pos[2], 
                 0, 0, 0, 1;

    view = translate * view;

    return view;
}

// rotate by z-axis
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f rotateMatrix;
    float radian=cal_radian(rotation_angle);
    float cosVal=std::cos(radian),sinVal=std::sin(radian);
    
    rotateMatrix<<cosVal,-sinVal,0,0,
                  sinVal,cosVal,0,0,
                  0,0,1,0,
                  0,0,0,1;
    
    model=rotateMatrix*model;
    return model;

}

//perspective projection 
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // caculate the t and r
    float t=std::tan(cal_radian(eye_fov/2.0))*std::abs(zNear),
    r=aspect_ratio*t;

    // generate the projection matrix
    Eigen::Matrix4f projectionMatrix;
    projectionMatrix<<zNear,0,0,0,
                      0,zNear,0,0,
                      0,0,zNear+zFar,-zNear*zFar,
                      0,0,1,0;
    projection=projectionMatrix*projection;

    Eigen::Matrix4f orthographic,orth_translate,orth_scale;
    orth_translate<<1,0,0,0,
                    0,1,0,0,
                    0,0,1,-1.0*(zNear+zFar)/2,
                    0,0,0,1;
    orth_scale<<1.0/r,0,0,0,
                0,1.0/t,0,0,
                0,0,2.0/(zNear-zFar),0,
                0,0,0,1;
    orthographic=orth_scale*orth_translate;
    projection=orthographic*projection;
    return projection;
}

// rotate by any axis 
Eigen::Matrix4f get_rotation(Eigen::Vector3f axis, float angle){
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f Rodrigues;
    Eigen::Matrix3f midMat;

    //caculate the dual matrix of axis
    Eigen::Matrix3f K;

    // normlize the axis
    float norm = sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
    axis[0]/=norm;
    axis[1]/=norm;
    axis[2]/=norm;
    K<<0,-axis(2),axis(1),
       axis(2),0,-axis(0),
       -axis(1),axis(0),0;
    
    float radian=cal_radian(angle);
    float cosVal=std::cos(radian),sinVal=std::sin(radian);

    // Rodrigues mat
    midMat=cosVal*Eigen::Matrix3f::Identity()+(1-cosVal)*axis*axis.transpose()+sinVal*K;

    Rodrigues=Eigen::Matrix4f::Identity();
    Rodrigues.block(0,0,3,3)=midMat;
    model=Rodrigues*model;
    return model;
   
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    // diy the axis
    float axis_x=0,axis_y=0,axis_z=1;

    if(argc>=5&&argv[1][0]=='s'){
        axis_x=std::stof(argv[2]);
        axis_y=std::stof(argv[3]);
        axis_z=std::stof(argv[4]);
    }
    else  if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else{
            return 0;
        }
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0; 
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        //r.set_model(get_model_matrix(angle));
        r.set_model(get_rotation(Eigen::Vector3f(axis_x,axis_y,axis_z),angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
