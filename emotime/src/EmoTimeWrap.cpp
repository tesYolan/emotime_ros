#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "SVMEmoDetector.h"
#include "BoostEmoDetector.h"
#include "matrix_io.h"
#include "FacePreProcessor.h"

#include "TrainingParameters.h"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <utility>
#include <map>
#include <iostream>
#include "std_msgs/String.h"

using namespace cv;
using namespace emotime;

using std::cout;
using std::endl;
using std::cerr;
using std::string;
using std::pair;

static const std::string OPENCV_WINDOW = "Image window";

class EmoTimeWrap
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  
  image_transport::Publisher image_pub_;
  ros::Publisher emotion_pub;
   std_msgs::String emotion_msg;
  string method;
  string config;
  string config_e; 
  Size size;
  int nwidths, nlambdas, nthetas;
  vector<string> classifier_paths;
  FacePreProcessor* preprocessor;
  EmoDetector* emodetector;
  string text;
  int fontFace;
  double fontScale;
  int thickness;  
  int lineType;
  bool bottomLeftOrigin;
//cv::Point textOrg(10, 130);
  Point textOrg;
  string emotionString;
  


public:
  EmoTimeWrap()
    : it_(nh_)
  {
    
 // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, 
      &EmoTimeWrap::imageCb, this);
      image_pub_ = it_.advertise("/image_converter/output_video", 1);
      emotion_pub = nh_.advertise<std_msgs::String>("chatter", 1000);
      method= "svm";
      config = "/home/lina/Desktop/emotime_final/emotime/resources/haarcascade_frontalface_cbcl1.xml";
      config_e = "/home/lina/Desktop/emotime_final/emotime/resources/haarcascade_eye.xml"; 
    size.width= 52;
    size.height= 52;
    nwidths= 1; 
    nlambdas= 5; 
    nthetas = 8; 
    
classifier_paths.push_back("/home/lina/svm/anger_vs_contempt_disgust_fear_happy_neutral_sadness_surprise_feats.xml");
classifier_paths.push_back("/home/lina/svm/contempt_vs_anger_disgust_fear_happy_neutral_sadness_surprise_feats.xml");
classifier_paths.push_back("/home/lina/svm/disgust_vs_anger_contempt_fear_happy_neutral_sadness_surprise_feats.xml");
classifier_paths.push_back("/home/lina/svm/fear_vs_anger_contempt_disgust_happy_neutral_sadness_surprise_feats.xml");
classifier_paths.push_back("/home/lina/svm/happy_vs_anger_contempt_disgust_fear_neutral_sadness_surprise_feats.xml");
classifier_paths.push_back("/home/lina/svm/neutral_vs_anger_contempt_disgust_fear_happy_sadness_surprise_feats.xml");
classifier_paths.push_back("/home/lina/svm/sadness_vs_anger_contempt_disgust_fear_happy_neutral_surprise_feats.xml");
classifier_paths.push_back("/home/lina/svm/surprise_vs_anger_contempt_disgust_fear_happy_neutral_sadness_feats.xml");

cout<<"Length of the classifiers: "<<classifier_paths.size()<<std::endl; 
     preprocessor = new FacePreProcessor(config, config_e, size.width,
          size.height, nwidths, nlambdas, nthetas);
    emodetector = new SVMEmoDetector(kCfactor, kMaxIteration, kErrorMargin);
    emodetector->init(classifier_paths);
    text = "Funny text inside the box";
   fontFace = FONT_HERSHEY_SCRIPT_SIMPLEX;
   fontScale = 2;
   thickness = 3;  
    textOrg.x = 10;
    textOrg.y = 130;
   int lineType=8;
   bool bottomLeftOrigin=false;
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~EmoTimeWrap()
  {
    cv::destroyWindow(OPENCV_WINDOW);
    
    delete emodetector;
    delete preprocessor;
  }


  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
	  
  cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    Mat img= cv_ptr->image; 
    Mat features; 
    
     bool canPreprocess = preprocessor->preprocess(img, features);
        if (!canPreprocess) {
          cout << " " << endl;
     }
     else{
        pair<Emotion,float> prediction = emodetector->predict(features);
        emotionString = emotionStrings(prediction.first);
        emotion_msg.data = emotionStrings(prediction.first);
         ROS_INFO("%s", emotion_msg.data.c_str());
    
}

    
    cv::putText(img, emotionString, textOrg, fontFace, fontScale, Scalar::all(255), thickness, lineType);
    // Output modified video stream
     cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    emotion_pub.publish(emotion_msg);
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc,  char* argv[]) {

  ros::init(argc, argv, "EmoTimeWrap");
  EmoTimeWrap etw;
  ros::spin();
 
  return 0;
}



