#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv4/opencv2/imgcodecs.hpp"
#include "opencv4/opencv2/highgui.hpp"
#include "opencv4/opencv2/objdetect.hpp"
#include "cv_bridge/cv_bridge.h"
#include "std_msgs/msg/float32.hpp"
#include <thread>
#include "interfaces_pkg/msg/point2f.hpp"
#include "opencv4/opencv2/dnn.hpp"

class number_recognizer_node : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr float_subscriber_;
    rclcpp::Subscription<interfaces_pkg::msg::Point2f>::SharedPtr point2f_subscriber_;
    rclcpp::Publisher<interfaces_pkg::msg::Point2f>::SharedPtr publisher_;
    double gap;
    cv::Point2f topLeft, topLeft_record, bottomRight, bottomRight_record;

    cv::dnn::Net loadOnnxModel(const std::string& modelPath) {
        cv::dnn::Net net = cv::dnn::readNetFromONNX(modelPath);
        if (net.empty()) {
            std::cout << "Failed to load the model." << std::endl;
            return cv::dnn::Net();
        } else {
            std::cout << "ONNX model loaded successfully." << std::endl;
            return net;
        }
    }


    std::vector<cv::Rect> postProcess(const cv::Mat& output, float confThreshold = 0.5) {
        std::vector<cv::Rect> boxes;
        for (int i = 0; i < output.rows; ++i) {
        float confidence = output.at<float>(i, 4);
        if (confidence > confThreshold) {
            int x = static_cast<int>(output.at<float>(i, 0));
            int y = static_cast<int>(output.at<float>(i, 1));
            int w = static_cast<int>(output.at<float>(i, 2));
            int h = static_cast<int>(output.at<float>(i, 3));
            boxes.push_back(cv::Rect(x, y, w, h));
        }
        }
        return boxes;
    }

    void num1cascadeFunc(cv::CascadeClassifier num1cascade, cv::Mat &imgGray, std::vector<cv::Rect> &num1)
    {
        num1cascade.detectMultiScale(imgGray, num1, 1.1, 5);
    }

    void num2cascadeFunc(cv::CascadeClassifier num2cascade, cv::Mat &imgGray, std::vector<cv::Rect> &num2)
    {
        num2cascade.detectMultiScale(imgGray, num2, 1.1, 5);
    }

    void num3cascadeFunc(cv::CascadeClassifier num3cascade, cv::Mat &imgGray, std::vector<cv::Rect> &num3)
    {
        num3cascade.detectMultiScale(imgGray, num3, 1.1, 5);
    }

    void num4cascadeFunc(cv::CascadeClassifier num4cascade, cv::Mat &imgGray, std::vector<cv::Rect> &num4)
    {
        num4cascade.detectMultiScale(imgGray, num4, 1.1, 5);
    }

public:

    explicit number_recognizer_node(const std::string &node_name) : Node(node_name)
    {
        RCLCPP_INFO(get_logger(), "number_recognizer_node launched");
        publisher_ = this->create_publisher<interfaces_pkg::msg::Point2f>("Point2f", 10);
        point2f_subscriber_ = this->create_subscription<interfaces_pkg::msg::Point2f>("Point2f_unselected", 10, [&](const interfaces_pkg::msg::Point2f::SharedPtr msg) ->void {
            this->topLeft.x = msg->topleft_x;
            this->topLeft.y = msg->topleft_y;
            this->bottomRight.x = msg->bottomright_x;
            this->bottomRight.y = msg->bottomright_y;
        });
        float_subscriber_ = this->create_subscription<std_msgs::msg::Float32>("float_data", 10, [&](const std_msgs::msg::Float32::SharedPtr msg) -> void { this->gap = msg->data; });
        subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("imageProcessed", 10, [&](const sensor_msgs::msg::Image::SharedPtr msg) -> void {            
            cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat imgCrop = imgPtr->image;
            if(imgCrop.empty()) {
                return;
            }

            cv::CascadeClassifier number_cascade[4];
            number_cascade[0].load("/home/ooofieee/Code/ws_0/src/armor_detector_pkg/resource/num1cascade.xml");
            number_cascade[1].load("/home/ooofieee/Code/ws_0/src/armor_detector_pkg/resource/num2cascade.xml");
            number_cascade[2].load("/home/ooofieee/Code/ws_0/src/armor_detector_pkg/resource/cascade.xml");
            number_cascade[3].load("/home/ooofieee/Code/ws_0/src/armor_detector_pkg/resource/num4cascade.xml");
            cv::cvtColor(imgCrop, imgCrop, cv::COLOR_BGR2GRAY);
            std::vector<cv::Rect> num1;
            std::vector<cv::Rect> num2;
            std::vector<cv::Rect> num3;
            std::vector<cv::Rect> num4;
            std::thread t1(std::bind(&number_recognizer_node::num1cascadeFunc, this, number_cascade[0], imgCrop, std::ref(num1)));
            std::thread t2(std::bind(&number_recognizer_node::num2cascadeFunc, this, number_cascade[1], imgCrop, std::ref(num2)));
            std::thread t3(std::bind(&number_recognizer_node::num3cascadeFunc, this, number_cascade[2], imgCrop, std::ref(num3)));
            std::thread t4(std::bind(&number_recognizer_node::num4cascadeFunc, this, number_cascade[3], imgCrop, std::ref(num4)));
            t1.join(); t2.join(); t3.join(); t4.join();

		    if (num1.size() != 0 || num2.size() != 0 || num3.size() != 0 || num4.size() != 0)
		    {            
                std::vector<std::vector<cv::Rect>> num = { num1,num2,num3,num4 };
                for (int i = 0; i < 4; i++)
		    	{
			    	for (int j = 0; j<(int)num[i].size();j++)
			    	{
			    		float areaNum = num[i][j].width * num[i][j].height;
			    		float areaImg = imgCrop.cols * imgCrop.rows;

				    	if (areaNum / areaImg > 0.3)
				    	{
                            float s1 = (topLeft.x*topLeft.x)+(topLeft.y*topLeft.y);
                            float s2 = (bottomRight.x*bottomRight.x)+(bottomRight.y*bottomRight.y);
                            if(s1>s2){
                                topLeft_record = bottomRight;
                                bottomRight_record = topLeft;	
                            }
                            else{
                                topLeft_record = topLeft;
                                bottomRight_record = bottomRight;
                            }

                            RCLCPP_INFO(get_logger(), "Fuck the World!");
                            auto msg = interfaces_pkg::msg::Point2f();
                            msg.topleft_x = topLeft_record.x;
                            msg.topleft_y = topLeft_record.y;
                            msg.bottomright_x = bottomRight_record.x;
                            msg.bottomright_y = bottomRight_record.y;
                            msg.i = i;
                            this->publisher_->publish(msg);
                        }
				    }

			    }
		    } 
        });
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<number_recognizer_node>("number_recognizer_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}