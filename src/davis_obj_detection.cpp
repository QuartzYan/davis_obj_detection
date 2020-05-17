#include "davis_obj_detection.h"

cv::Scalar colorTab[] = {
    cv::Scalar(0,0,0),
    cv::Scalar(0,255,0),
    cv::Scalar(255,0,0),
    cv::Scalar(0,255,255),
    cv::Scalar(255,0,255)
};

DavisObjDetection::DavisObjDetection()
  :K(2),point_count_(0)
{
  center_point_ = cv::Point(0, 0);

  image_sub_ = nh_.subscribe("stitch_image", 1, &DavisObjDetection::imageCallback, this);
  event_image_sub_ = nh_.subscribe("stitch_event_image", 1, &DavisObjDetection::eventImageCallback, this);

  image_transport::ImageTransport it_(nh_);
  image_pub_ = it_.advertise("obj_image", 1);
  event_image_pub_ = it_.advertise("obj_event_image", 1);
}

DavisObjDetection::~DavisObjDetection()
{

}

void DavisObjDetection::imageCallback(const sensor_msgs::Image::ConstPtr& image)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(image);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat srcImg;
  cv::cvtColor(cv_ptr->image, srcImg, CV_RGB2BGR);

  if(point_count_ > 50)
  {
    cv::circle(srcImg, center_point_, 100, cv::Scalar(255, 255, 255));
  }

  //pub
  cv_bridge::CvImage out_cv_image;
  srcImg.copyTo(out_cv_image.image);
  out_cv_image.encoding = "bgr8";
  image_pub_.publish(out_cv_image.toImageMsg());
}

void DavisObjDetection::eventImageCallback(const sensor_msgs::Image::ConstPtr& image)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(image);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat srcImg;
  cv::cvtColor(cv_ptr->image, srcImg, CV_RGB2GRAY);

  //init
  int width = srcImg.cols;
  int height = srcImg.rows;
  int channels = srcImg.channels();
  int sampleCount = width * height;
  cv::Mat labels, center;
  cv::Mat data(sampleCount, channels, CV_32F, cv::Scalar::all(0));
  cv::Mat sampleData = srcImg.reshape(channels, sampleCount);
  sampleData.convertTo(data, CV_32F);
  
  //run K-means
  cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 10, 0.1);
  cv::kmeans(data, K, labels, criteria, 3, cv::KMEANS_PP_CENTERS, center);

  //result
  cv::Mat result = cv::Mat(srcImg.size(), CV_8UC3, cv::Scalar::all(0));
  int SX = 0, SY = 0;
  point_count_ = 0;
  for (int i = 0; i < srcImg.rows; i++)
  {
    for (int j = 0; j < srcImg.cols; j++)
    {
      int index = i * width + j;
      int label = labels.at<int>(index);
      if (label)
      {
        SX += j;
        SY += i;
        point_count_++;
      }
    }
  }

  if(point_count_ > 50)
  {
    center_point_.x = SX / point_count_;
    center_point_.y = SY / point_count_;
    cv::circle(cv_ptr->image, center_point_, 100, cv::Scalar(255, 255, 255));
  }

  //pub
  cv_bridge::CvImage out_cv_image;
  cv_ptr->image.copyTo(out_cv_image.image);
  out_cv_image.encoding = "bgr8";
  event_image_pub_.publish(out_cv_image.toImageMsg());
}