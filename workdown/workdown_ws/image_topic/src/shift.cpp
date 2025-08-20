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
Mat mask_light(const Mat& bgr){
    Mat hsv_img,mask_red1,mask_red2,mask_blue,out_img;
    cvtColor(bgr,hsv_img,COLOR_BGR2HSV);
    inRange(hsv_img,Scalar(0,120,70),Scalar(10,255,255),mask_red1);
    inRange(hsv_img,Scalar(156,120,70),Scalar(180,255,255),mask_red2);
    inRange(hsv_img,Scalar(100,120,70),Scalar(124,255,255),mask_blue);
    out_img = mask_red1 | mask_red2 | mask_blue;
    cv::morphologyEx(out_img, out_img, cv::MORPH_CLOSE,cv::getStructuringElement(cv::MORPH_RECT, {3, 3}), {-1, -1}, 2);
    return out_img;
};
//2、灯条检测
std::vector<RotatedRect> detect_light_bar(const Mat& img_in,const Mat& img_out){
    std::vector<std::vector<Point>> contours;
    findContours(img_in,contours,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
    std::vector<RotatedRect> bars;
    for (const auto& c : contours)
    {
        if (c.size() < 5) continue;
        cv::RotatedRect r = cv::minAreaRect(c);
        float h = std::max(r.size.height, r.size.width);
        float w = std::min(r.size.height, r.size.width);
        if (h / w < 2.0f || h / w > 8.0f) continue;
        if (h < img_out.rows * 0.02f) continue;
        bars.push_back(r);
    }
    return bars;
};
//3、灯条配对（返回四边形角点）
std::vector<Point2f> pairLightBars(const std::vector<cv::RotatedRect>& bars,const cv::Size& img_size){
    const float diag = norm(Vec2f(img_size.width, img_size.height));
    for (size_t i = 0; i < bars.size(); ++i)
    {
        for (size_t j = i + 1; j < bars.size(); ++j)
        {
            const auto& r1 = bars[i];
            const auto& r2 = bars[j];
            float len1 = std::max(r1.size.height, r1.size.width);
            float len2 = std::max(r2.size.height, r2.size.width);
            if (std::fabs(len1 - len2) / std::max(len1, len2) > 0.4f) continue;

            float d = cv::norm(r1.center - r2.center);
            if (d < len1 || d > len1 * 4 || d > diag * 0.6f) continue;

            // 生成四边形角点
            auto vec = [](const cv::RotatedRect& r) -> cv::Point2f {
                cv::Point2f pts[4];
                r.points(pts);
                cv::Point2f v =
                    (r.size.height > r.size.width) ? (pts[2] - pts[1]) : (pts[1] - pts[0]);
                return v * 0.5f;
            };
            std::vector<cv::Point2f> corners = {
                r1.center - vec(r1),  // 灯条1 上
                r1.center + vec(r1),  // 灯条1 下
                r2.center + vec(r2),  // 灯条2 下
                r2.center - vec(r2)   // 灯条2 上
            };
            return corners;
        }
    }
    return {};
};
//4、pnp解算
bool solveArmorPose(const std::vector<cv::Point2f>& img_corners,
                    cv::Mat& rvec, cv::Mat& tvec)
{
    if (img_corners.size() != 4) return false;
    cv::solvePnP(OBJ_PTF, img_corners, K, D, rvec, tvec);
    return true;
};
// 5. 相机坐标 → 机器人坐标
cv::Point3f cam2Robot(const cv::Mat& rvec, const cv::Mat& tvec)
{
    cv::Mat R_obj2cam;
    cv::Rodrigues(rvec, R_obj2cam);
    cv::Mat t_obj2robot = R_CAM2ROBOT * tvec + T_CAM2ROBOT;
    return cv::Point3f(t_obj2robot.at<float>(0),
                       t_obj2robot.at<float>(1),
                       t_obj2robot.at<float>(2));
};
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
        auto bars = detect_light_bar(cv_img,cv_img_in);
        auto corners = pairLightBars(bars, cv_img_in.size());
        Mat rvec, tvec;
        Point3f pt3d{NAN, NAN, NAN};
        if (!corners.empty() && solveArmorPose(corners, rvec, tvec))
            pt3d = cam2Robot(rvec, tvec);
            RCLCPP_INFO(this->get_logger(),"%f,%f,%f",pt3d.x,pt3d.y,pt3d.z);
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