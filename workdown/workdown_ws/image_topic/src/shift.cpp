#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/msg/image.hpp"
#include "aim_interfaces/msg/aim_info.hpp"
using namespace cv;

//内参
static const Mat K = (Mat_<double>(3,3) << 1462.3697,0,398.59394,
0,1469.68358,110.68997,
0,0,1);
//畸变矩阵
static const Mat D = (Mat_<double>(5,1) << 0.003518, -0.311778,-0.016581,0.023682,0.0000);

//将灯条范围作为装甲板尺寸，则装甲板尺寸：
static const float a_height = 0.08f;
static const float a_weight = 0.16f;
//装甲板四个角点坐标
static const std::vector<Point3f> OBJ_PTF = {{-a_weight/2,a_height/2,0},{a_weight/2,a_height/2,0},{a_weight/2,-a_height/2,0},{-a_weight/2,-a_height/2,0}};

//相机坐标系到机器人坐标系的转换
//旋转矩阵
static const Mat R_CAM2ROBOT = []{
    Mat R;
    Rodrigues(Vec3f(0,60,20)*float(CV_PI/180.f),R);
    return R;
}();
//平移向量
static const Mat T_CAM2ROBOT = (Mat_<float>(3,1) << 0.08f,0.0f,0.05f);

//副函数
//提取灯条函数
//匹配装甲板函数
//区分装甲板函数

//主函数



class Shift : public rclcpp::Node{
public:
    Shift(std::string name):Node(name){
        RCLCPP_INFO(this->get_logger(),"%s节点已经启动。",name.c_str());
        shift_subscribe = this->create_subscription<sensor_msgs::msg::Image>("sensor",10,std::bind(&Shift::sensor_callback,this,std::placeholders::_1));
        shift_publisher = this->create_publisher<aim_interfaces::msg::AimInfo>("array",10);
        shift_timer = this->create_wall_timer(std::chrono::milliseconds(500),std::bind(&Shift::aim_timer_callback,this));
    }
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr shift_subscribe;
    rclcpp::Publisher<aim_interfaces::msg::AimInfo>::SharedPtr shift_publisher;
    rclcpp::TimerBase::SharedPtr shift_timer;
    void sensor_callback(const sensor_msgs::msg::Image::SharedPtr ros_img){
        Mat cv_img_in = cv_bridge::toCvCopy(ros_img, "bgr8")->image;
        namedWindow("Window1",WINDOW_AUTOSIZE);
        imshow("Window1",cv_img_in);
        waitKey(1);
    }

    void aim_timer_callback(){

    }

};

int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<Shift>("Shift");

    rclcpp::spin(node);
    rclcpp::shutdown();
}