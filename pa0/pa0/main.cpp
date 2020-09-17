#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>

Eigen::Matrix3f RotateTransformation(float angle){
    const float pi=std::acos(-1);
    float cosVal=std::cos(angle*pi/180.0),sinVal=std::sin(angle*pi/180.0);
    Eigen::Matrix3f res;
    res<<cosVal,-sinVal,0,sinVal,cosVal,0,0,0,1;
    return res;
}

Eigen::Matrix3f TranslateTransformation(float x,float y){
    Eigen::Matrix3f res;
    res<<1,0,x,0,1,y,0,0,1;
    return res;
}


int main(){
    std::cout<<"Please input the point:";
    float x,y;
    std::cin>>x>>y;
    Eigen::Vector3f point;
    point<<x,y,1;
    std::cout<<TranslateTransformation(1,2)*RotateTransformation(45)*point;



    return 0;
}