#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/msg/image.hpp"
#include "aim_interfaces/msg/aim_info.hpp"
using namespace cv;

//内参
static const Mat K = (Mat_<double>(3,3) << 1462.3697,0,398.59394,0,1469.68385,110.68997,0,0,1);
//畸变矩阵
static const Mat D = (Mat_<double>(5,1) << 0.003518, -0.311778,-0.016581,0.023682,0.0000);

//将灯条范围作为装甲板尺寸，则装甲板尺寸：
static const float a_height = 0.08f;
static const float a_weight = 0.16f;

//相机坐标系到机器人坐标系的转换
//相机->机器人旋转矩阵
static const Mat R_CAM2ROBOT = []{
    Mat R;
    Rodrigues(Vec3f(0,60,20)*float(CV_PI/180.f),R);
    return R;
}();
//相机->机器人平移向量
static const Mat T_CAM2ROBOT = (Mat_<float>(3,1) << 0.08f,0.0f,0.05f);

//世界坐标系下坐标
static const std::vector<Point3f> OBJ_PTF = {{-a_weight/2,a_height/2,0},{a_weight/2,a_height/2,0},{a_weight/2,-a_height/2,0},{-a_weight/2,-a_height/2,0}};
//图像坐标
//1、分割红/蓝灯条
Mat mask_light(const Mat& img_in){
    //声明hsv图像、红/蓝滤波、输出图像
    Mat hsv_img,mask_red1,mask_red2,mask_blue,out_img;
    //转换为hsv图像
    cvtColor(img_in,hsv_img,COLOR_BGR2HSV);
    //定义红蓝滤波
    inRange(hsv_img,Scalar(0,120,70),Scalar(10,255,255),mask_red1);
    inRange(hsv_img,Scalar(156,120,70),Scalar(180,255,255),mask_red2);
    inRange(hsv_img,Scalar(100,120,70),Scalar(124,255,255),mask_blue);
    //定义输出图像
    out_img = mask_red1 | mask_red2 | mask_blue;
    //膨胀后腐蚀
    morphologyEx(out_img, out_img, MORPH_CLOSE,getStructuringElement(cv::MORPH_RECT, {3, 3}), {-1, -1}, 2);
    return out_img;
};
//2、灯条检测
std::vector<RotatedRect> detect_lightbar(const Mat& img_in){
    //轮廓容器
    std::vector<std::vector<Point>> contours;
    //查找轮廓
    findContours(img_in,contours,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
    //声明灯带
    std::vector<RotatedRect> bars;
    //遍历轮廓
    for (const auto& c : contours)
    {
        //轮廓点数少无法拟合，应跳过
        if (c.size() < 5) continue;
        //矩形拟合
        RotatedRect r = minAreaRect(c);
        bars.push_back(r);
    }
    return bars;
};
//3、绘制装甲板轮廓
std::vector<std::vector<Point2f>> draw_armors(const std::vector<RotatedRect>& bars){
    //声明装甲板数组
    std::vector<std::vector<Point2f>> armors;
    
    //暴力进行灯条逐一配对
    for(size_t i = 0; i < bars.size();++i){
        for(size_t j = i+1; j < bars.size();++j){
            const auto& r1 = bars[i];
            const auto& r2 = bars[j];
            float len1 = std::max(r1.size.height, r1.size.width);
            float len2 = std::max(r2.size.height, r2.size.width);
            if (std::fabs(len1 - len2) / std::max(len1, len2) > 0.4f){
                continue;
            }
            float d = norm(r1.center-r2.center);
            if(d > len1+len2){
                continue;
            }
            //角度筛选
            if(std::fabs(r1.angle - r2.angle)>20.0f){
                continue;
            }
            //生成四边形角点
            auto vec = [](const RotatedRect& r) -> Point2f{
                Point2f p[4];
                r.points(p);
                Point2f v = (r.size.width > r.size.height) ? (p[1]-p[0]) : (p[2]-p[1]);
                return v*0.5;
            };
            
            std::vector<Point2f> corners = {
                r1.center + vec(r1),
                r2.center + vec(r2),
                r2.center - vec(r2),
                r1.center - vec(r1)
            };

            armors.emplace_back(std::move(corners));
        }
    }
    return armors;
};
//4、pnp解算
// 5. 相机坐标 → 机器人坐标
//转换为机器人坐标系坐标



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
        Mat cv_img  = mask_light(cv_img_in);
        auto bars = detect_lightbar(cv_img);
        std::vector<std::vector<Point2f>> all_armors = draw_armors(bars);
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