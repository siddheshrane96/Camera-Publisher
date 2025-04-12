#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

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

        // image_transport::ImageTransport it(shared_from_this());
        // m_image_publisher = it.advertise("/camera/image_raw", 1);
        auto qos = rclcpp::QoS(rclcpp::SensorDataQoS());
        m_image_publisher = image_transport::create_publisher(this, "/camera/image_raw", qos.get_rmw_qos_profile());


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

        m_image_publisher.publish(image_msg);
    }
    
    cv::VideoCapture m_cap;
    image_transport::Publisher m_image_publisher;
    rclcpp::TimerBase::SharedPtr m_timer;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto webcam_publisher_node = std::make_shared<WebcamPublisher>();
    rclcpp::spin(webcam_publisher_node);
    rclcpp::shutdown();
    return 0;
}

