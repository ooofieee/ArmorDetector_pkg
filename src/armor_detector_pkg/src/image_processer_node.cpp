#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv4/opencv2/imgcodecs.hpp"
#include "opencv4/opencv2/highgui.hpp"
#include "cv_bridge/cv_bridge.h"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/float32.hpp"
#include "interfaces_pkg/msg/point2f.hpp"
#include"opencv4/opencv2/opencv.hpp"
#include <cmath>

using namespace cv;
using namespace std;

class image_processer_node : public rclcpp::Node
{
private:
	rclcpp::Publisher<interfaces_pkg::msg::Point2f>::SharedPtr publisher_point2f;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr side_publisher_;
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr float_publisher_;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
	Mat frame, frame_undistorted, frameHSV, maskBlur, maskHSV, maskCanny, maskDilHSV, dst, imgCrop, imgGray, imgHSV;
	Mat camera_matrix, distortion_coeffs;
	Point2f roi_topLeft = Point2f(0.0, 0.0);
	Point2f roi_bottomRight = Point2f(frame.cols, frame.rows);
	// vector<vector<int>> LightBarHSV = {{0, 179, 0, 255, 250, 255}, {0, 164, 0, 61, 245, 255}, {0, 179, 0, 255, 236, 255}, {16, 32, 0, 162, 191, 255}};
	vector<Mat> channels;

	void loadCameraParameters()
    {
        camera_matrix = (cv::Mat_<double>(3, 3) << 1795.48075, 0, 719.15967,
                                                    0, 1788.97397, 554.12545,
                                                     0, 0, 1);
        distortion_coeffs = (cv::Mat_<double>(1, 5) << -0.073464, 0.128799, 0.001334, 0.001541, 0.000000);
    }

public:
	explicit image_processer_node(const std::string &node_name) : Node(node_name)
	{
		RCLCPP_INFO(get_logger(), "image_processer_node launched");
		loadCameraParameters();
		side_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("side_image", 10);
		publisher_point2f = this->create_publisher<interfaces_pkg::msg::Point2f>("Point2f_unselected", 10);
		publisher_ = this->create_publisher<sensor_msgs::msg::Image>("imageProcessed", 10);
		float_publisher_ = this->create_publisher<std_msgs::msg::Float32>("float_data", 10);
		subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("image", 10, [&](const sensor_msgs::msg::Image::SharedPtr msg) -> void {
			RCLCPP_INFO(get_logger(), "frame received");
            cv_bridge::CvImagePtr framePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            frame = framePtr->image;
			undistort(frame, frame_undistorted, camera_matrix, distortion_coeffs);
			Mat frame_split;
			split(frame_undistorted,channels);
			frame_split = channels.at(2) - channels.at(0);
			Mat binBrightImg;
			threshold(frame_split, binBrightImg, 120, 255, cv::THRESH_BINARY);
			Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9));
			dilate(binBrightImg, binBrightImg, element);
			auto side_msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", binBrightImg).toImageMsg();
			side_publisher_->publish(*side_msg_);

	        Mat kernelHSV = getStructuringElement(MORPH_RECT, Size(5, 5));
	        vector<vector<Point>> contoursHSV;
	        vector<Vec4i> hierarchy;

            Point2f topLeft, bottomRight, center;
            double gap,  averageHeight;
			findContours(binBrightImg, contoursHSV, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
			vector<RotatedRect> boundRect(contoursHSV.size());
			int noise = 0;

			for (int i = 0; i < (int)contoursHSV.size(); i++)
			{
				if (contoursHSV[i].size() >= 6)
				{
					boundRect[i] = fitEllipse(contoursHSV[i]);
				}
				else
				{
					noise++;
				}
			}

            for (int i = 0; i < (int)boundRect.size() - noise; i++)
			{
				for (int j = i + 1; j < (int)boundRect.size() - noise; j++)
				{
					if (boundRect[j].size.height / boundRect[j].size.width > 2 && boundRect[j].size.height / boundRect[j].size.width < 5)
					{
						averageHeight = (boundRect[i].size.height + boundRect[j].size.height) / 2;
						gap = boundRect[i].center.x - boundRect[j].center.x;
						center = (boundRect[i].center + boundRect[j].center) / 2;
						topLeft = center - Point2f(gap / 2, gap /2 );
						bottomRight = center + Point2f(gap / 2, gap /2 );

						if (topLeft.y >= 0 && topLeft.x >= 0 && topLeft.y <= frame.rows && topLeft.x <= frame.cols && bottomRight.y <= frame.rows && bottomRight.x <= frame.cols && bottomRight.y >= 0 && bottomRight.x > 0)
						{
							if (boundRect[i].center.y / boundRect[j].center.y < 1.1 && boundRect[i].center.y / boundRect[j].center.y > 0.9)
							{
								if(std::fabs(boundRect[i].angle - boundRect[j].angle) < 15)
								{
									if ((boundRect[i].size.height / boundRect[j].size.height < 1.1 && boundRect[i].size.height / boundRect[j].size.height > 0.9) || (boundRect[i].size.width / boundRect[j].size.width < 1.1 && boundRect[i].size.width / boundRect[j].size.width > 0.9))
									{
										Rect roi(topLeft, bottomRight);
										imgCrop = frame(roi);
										auto msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", imgCrop).toImageMsg();
										auto float_msg_ = std_msgs::msg::Float32();
										auto point_msg_ = interfaces_pkg::msg::Point2f();
										point_msg_.topleft_x = topLeft.x;
										point_msg_.topleft_y = topLeft.y;
										point_msg_.bottomright_x = bottomRight.x;
										point_msg_.bottomright_y = bottomRight.y;
										float_msg_.data = gap;
										this->publisher_point2f->publish(point_msg_);
										this->publisher_->publish(*msg_);
										this->float_publisher_->publish(float_msg_);
									}
								}
							}
						}
					}
				}
			}
			RCLCPP_INFO(get_logger(), "frame processed"); });
	}
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<image_processer_node>("image_processer_node");
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}