#include "FrameProcessing.hpp"

FrameProcessing::FrameProcessing(){}


void FrameProcessing::trapezoid_roi(const Mat& frame, string trap_bottom_width_s, string trap_top_width_s, string trap_height_s, string car_hood_s)
{
    int width = (int)frame.size().width;
    int height = (int)frame.size().height;

    trap_bottom_width = stof(trap_bottom_width_s);
    trap_top_width = stof(trap_top_width_s);
    trap_height = stof(trap_height_s);
    car_hood = stof(car_hood_s);

    original_roi.push_back(Point2f( (width * (1 - trap_bottom_width)) / 2, height - car_hood));
    original_roi.push_back(Point2f( (width * (1 - trap_top_width)) / 2, height - height * trap_height));
    original_roi.push_back(Point2f( width - (width * (1 - trap_top_width)) / 2, height - height * trap_height));
    original_roi.push_back(Point2f( width - (width * (1 - trap_bottom_width)) / 2, height - car_hood));

    warped_roi.push_back(Point2f( ( width * (1 - trap_bottom_width)) / 2  , (float)height));
    warped_roi.push_back(Point2f( ( width * (1 - trap_bottom_width)) / 2, 0.0));
    warped_roi.push_back(Point2f( width - (width * (1 - trap_bottom_width)) / 2, 0.0));
    warped_roi.push_back(Point2f( width - (width * (1 - trap_bottom_width)) / 2, (float)height));

    return;
}

void FrameProcessing::color_filter(Mat& frame, Mat& filtered_image, string& white_range_s)
{
    Mat mask1;
    Mat mask2;
    Mat image_hsv;
    Mat white_image;
    Mat yellow_image;

    white_range = stoi(white_range_s);

    cv::inRange(frame, Scalar(white_range,white_range,white_range), Scalar(255,255,255), mask1);

    cvtColor(frame, image_hsv, COLOR_RGB2HSV);
    cv::inRange(image_hsv, Scalar(90,70,100), Scalar(110,255,255), mask2);

    cv::bitwise_and(frame, frame, white_image, mask1);
    
    cv::bitwise_and(frame, frame, yellow_image, mask2); 
    
    addWeighted(white_image, 1., yellow_image, 1., 0., filtered_image);

    return;
}

 void FrameProcessing::calculate_sobel(Mat& frame,Mat& sobel_output)
 {
   int depth = CV_16S;
   Mat gray;
   Mat grad_x, grad_y, abs_grad_x, abs_grad_y;
   int scale = 1;
   int delta = 0;

   GaussianBlur(frame, frame, Size(3,3), 0, 0, BORDER_DEFAULT);

   cvtColor(frame, gray, COLOR_BGR2GRAY);

   Sobel(gray, grad_x, depth, 1, 0, 3, scale, delta, BORDER_DEFAULT);
   convertScaleAbs(grad_x, abs_grad_x);

   //Sobel(gray, grad_y, depth, 0, 1, 3, scale, delta, BORDER_DEFAULT);
   //convertScaleAbs(grad_y, abs_grad_y);

   //addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0 , sobel_output);
   sobel_output = abs_grad_x;

   return;
 }


void FrameProcessing::perspective_transform(const Mat& filtered_image_gray, Mat& binary_warped)
{
  Mat binary_threshold;
  Mat M(2,4,CV_32FC2);

  M = getPerspectiveTransform(original_roi, warped_roi);

  binary_threshold =  Mat::zeros(filtered_image_gray.rows, filtered_image_gray.cols, CV_8UC3);
  threshold(filtered_image_gray, binary_threshold, 0, 255, THRESH_BINARY);

  warpPerspective(binary_threshold, binary_warped, M, filtered_image_gray.size(), INTER_LINEAR);

  return;
}
