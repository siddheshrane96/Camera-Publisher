#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <camera_info_manager/camera_info_manager.hpp>

class WebcamPublisher:
    public rclcpp::Node
{
public:
    WebcamPublisher():
        Node("webcam_publisher"),
        m_cap(0)
    {
        if (!m_cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open webcam");
            rclcpp::shutdown();
            return;
        }

        // Camera info parameters
        this->declare_parameter<std::string>("camera_name","webcam");
        this->declare_parameter<std::string>("camera_info_url", "");
        m_camera_name = get_parameter("camera_name").as_string();
        m_camera_info_url = get_parameter("camera_info_url").as_string();

        m_camera_info_manager = std::make_shared<camera_info_manager::CameraInfoManager>(this, m_camera_name, m_camera_info_url);

        // image_transport::ImageTransport it(shared_from_this());
        // m_image_publisher = it.advertise("/camera/image_raw", 1);
        auto qos = rclcpp::QoS(rclcpp::SensorDataQoS());
        m_image_publisher = image_transport::create_publisher(this, "/camera/image_raw", qos.get_rmw_qos_profile());
        m_cam_info_publisher = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/camera_info", qos);

        m_timer = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&WebcamPublisher::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Webcam publisher initialized.");
        
    }
private:
    void timer_callback()
    {
        cv::Mat frame;
        m_cap >> frame;
        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty frame captured");
            return;
        }

        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "camera_link";

        sensor_msgs::msg::Image::SharedPtr image_msg = 
            cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();

        sensor_msgs::msg::CameraInfo camera_info_msg = m_camera_info_manager->getCameraInfo();
        camera_info_msg.header = header;

        m_image_publisher.publish(image_msg);
        m_cam_info_publisher->publish(camera_info_msg);
    }
    
    image_transport::Publisher m_image_publisher;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr m_cam_info_publisher;
    std::shared_ptr<camera_info_manager::CameraInfoManager> m_camera_info_manager;
    rclcpp::TimerBase::SharedPtr m_timer;
    
    cv::VideoCapture m_cap;
    std::string m_calibration_path{"/home/siddhesh/Downloads/camera_publisher_ws/src/webcam_publisher_cpp/calibration/calibration.yaml"};
    std::string m_camera_name;
    std::string m_camera_info_url;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto webcam_publisher_node = std::make_shared<WebcamPublisher>();
    rclcpp::spin(webcam_publisher_node);
    rclcpp::shutdown();
    return 0;
}

