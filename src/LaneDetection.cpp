#include "../include/LaneDetection.hpp"

LaneDetection::LaneDetection()
{
  trap_bottom_width = 0.7f; 
  trap_top_width = 0.1f; 
  trap_height = 0.38f;
  car_hood = 50;
  first_frame = true;

}

void LaneDetection::color_filter(Mat& filtered_image)
{
    Mat mask1;
    Mat mask2;
    Mat image_hsv;
    Mat white_image;
    Mat yellow_image;

    cv::inRange(frame, Scalar(190,190,190), Scalar(255,255,255), mask1);

    cvtColor(frame, image_hsv, COLOR_RGB2HSV);
    cv::inRange(image_hsv, Scalar(90,70,100), Scalar(110,255,255), mask2);

    cv::bitwise_and(frame, frame, white_image, mask1);
    
    cv::bitwise_and(frame, frame, yellow_image, mask2); 
    
    addWeighted(white_image, 1., yellow_image, 1., 0., filtered_image);
    
}

 void LaneDetection::calculate_sobel(Mat& sobel_output)
 {
   int dx, dy;
   int depth = CV_16S;
   Mat gray;
   Mat grad_x, grad_y, abs_grad_x, abs_grad_y;
   int scale = 1;
   int delta = 0;

   GaussianBlur(frame, frame, Size(3,3), 0, 0, BORDER_DEFAULT);

   cvtColor(frame, gray, COLOR_BGR2GRAY);

   Sobel(gray, grad_x, depth, 1, 0, 3, scale, delta, BORDER_DEFAULT);
   convertScaleAbs(grad_x, abs_grad_x);

   Sobel(gray, grad_y, depth, 0, 1, 3, scale, delta, BORDER_DEFAULT);
   convertScaleAbs(grad_y, abs_grad_y);

   addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0 , sobel_output);

   return;

 }


void LaneDetection::trapezoid_roi()
{
    int width = (int)frame.size().width;
    int height = (int)frame.size().height;

    original_roi.push_back(Point2f( (width * (1 - trap_bottom_width)) / 2, height - car_hood));
    original_roi.push_back(Point2f( (width * (1 - trap_top_width)) / 2, height - height * trap_height));
    original_roi.push_back(Point2f( width - (width * (1 - trap_top_width)) / 2, height - height * trap_height));
    original_roi.push_back(Point2f( width - (width * (1 - trap_bottom_width)) / 2, height - car_hood));

    warped_roi.push_back(Point2f( ( width * (1 - trap_bottom_width)) / 2 , (float)height));
    warped_roi.push_back(Point2f( ( width * (1 - trap_bottom_width)) / 2, 0.0));
    warped_roi.push_back(Point2f( width - (width * (1 - trap_bottom_width)) / 2, 0.0));
    warped_roi.push_back(Point2f( width - (width * (1 - trap_bottom_width)) / 2, (float)height));

    return;

}

void LaneDetection::perspective_transform(const Mat& filtered_image_gray, Mat& binary_warped)
{
  Mat binary_threshold;
  Mat M(2,4,CV_32FC2);
  

  M = getPerspectiveTransform(original_roi, warped_roi);
  
  binary_threshold =  Mat::zeros(filtered_image_gray.rows, filtered_image_gray.cols, CV_8UC3);
  threshold(filtered_image_gray, binary_threshold, 0, 255, THRESH_BINARY);

  warpPerspective(binary_threshold, binary_warped, M, filtered_image_gray.size(), INTER_LINEAR);
  

}

void LaneDetection::get_histogram(Mat const& binary_warped, Mat& histogram)
{
  cv::Mat half_image = binary_warped(cv::Rect(0, binary_warped.rows / 2, binary_warped.cols, binary_warped.rows / 2));
	cv::reduce(half_image / 255, histogram, 0, REDUCE_SUM, CV_32FC1);

	return;
}

void LaneDetection::calculate_lane_histogram(const Mat& histogram, Point& left_peak, Point& right_peak)
{
  int midpoint;
  Mat left_x_base;
  Mat right_x_base;
  midpoint = histogram.cols / 2;
  left_x_base = histogram.colRange(0,midpoint);
  right_x_base = histogram.colRange(midpoint,histogram.cols);

  minMaxLoc(left_x_base, NULL, NULL, NULL, &left_peak);
  minMaxLoc(right_x_base,NULL, NULL, NULL, &right_peak);

  right_peak = right_peak + Point(midpoint, 0);
}

void LaneDetection::sliding_window(Mat& binary_warped, Point& left_peak, Point& right_peak, Mat& output_image, vector<Window>& left_boxes, vector<Window>& right_boxes)
{
  int N_windows;
  int window_width;
  int window_height;

  Mat gray_tmp;

  N_windows = 9;
  window_width = 100;
  window_height = binary_warped.rows / N_windows;

  gray_tmp = binary_warped.clone();

  Window window_left(binary_warped, left_peak.x, binary_warped.rows - window_height, window_width, window_height, 50);
  Window window_right(binary_warped, right_peak.x, binary_warped.rows - window_height, window_width, window_height, 50);

  cvtColor(binary_warped, output_image, COLOR_GRAY2RGB);

  for(int i = 0; i < N_windows; i++)
  {
      rectangle(output_image, window_left.get_bottom_left_point(), window_left.get_top_right_point(), Scalar(0,255, 0), 2);
      rectangle(output_image, window_right.get_bottom_left_point(), window_right.get_top_right_point(), Scalar(0,255, 0), 2);

      left_boxes.push_back(window_left);
      right_boxes.push_back(window_right);
      
      window_left = window_left.get_next_window(gray_tmp);
      window_right = window_right.get_next_window(gray_tmp);

  }

}

Mat LaneDetection::polyfit_windows(vector<Window> const& windows)
{
	int n = (int)windows.size();

	vector<Mat> x_mat, y_mat;

	x_mat.reserve(n);
	y_mat.reserve(n);

	Mat x_temp, y_temp;

	for (Window const& window : windows)
  {
		window.get_indices(x_temp, y_temp);
		x_mat.push_back(x_temp);
		y_mat.push_back(y_temp);
	}

	Mat xs, ys;

	vconcat(x_mat, xs);
	vconcat(y_mat, ys);

	Mat fit = Mat::zeros(3, 1, CV_32FC1);

	polyfit(ys, xs, fit, 2);

	return fit;

}

void LaneDetection::calculate_lane_fit_next_frame(vector<Point2f> non_zero, Mat& lane_fit, vector<float>& xs, vector<float>& ys, int margin) 
{
  float x, y, left_x, right_x;

  for(auto const& pt: non_zero)
  {
    x = pt.x;
    y = pt.y;

    left_x = lane_fit.at<float>(2, 0) * y * y + lane_fit.at<float>(1, 0) * y + lane_fit.at<float>(0, 0) - margin;
    right_x = lane_fit.at<float>(2, 0) * y * y + lane_fit.at<float>(1, 0) * y + lane_fit.at<float>(0, 0) + margin;

    if(x > left_x && x < right_x)
    {
      xs.push_back(x);
      ys.push_back(y);
    }

  }

  return;

}

void LaneDetection::non_sliding_window(Mat& binary_warped, Mat& left_fit, Mat& right_fit, Mat& new_left_fit,  Mat& new_right_fit, int margin)
{
  vector<Point2f> non_zero;
  cv::findNonZero(binary_warped, non_zero);

  vector<float> left_xs, left_ys, right_xs, right_ys;

  calculate_lane_fit_next_frame(non_zero, left_fit, left_xs, left_ys, margin);
  calculate_lane_fit_next_frame(non_zero, right_fit, right_xs, right_ys, margin);

  new_left_fit = left_fit;
  new_right_fit = right_fit;

  if(!left_fit.empty())
  {
    new_left_fit = cv::Mat::zeros(3,1, CV_32FC1);
    Mat xs(left_xs, CV_32FC1);
    Mat ys(left_ys, CV_32FC1);
    polyfit(ys, xs, new_left_fit, 2);

  }  

  if(!right_fit.empty())
  {
    new_right_fit = cv::Mat::zeros(3,1, CV_32FC1);
    Mat xs(right_xs, CV_32FC1);
    Mat ys(right_ys, CV_32FC1);
    polyfit(ys, xs, new_right_fit, 2);
    
  } 

  return;

}

void LaneDetection::polyfit(const Mat& src_x, const Mat& src_y, Mat& dst, int order)
{

  CV_Assert((src_x.rows > 0) && (src_y.rows > 0) && (src_x.cols == 1) && (src_y.cols == 1)
		&& (dst.cols == 1) && (dst.rows == (order + 1)) && (order >= 1));

	Mat X;
	X = Mat::zeros(src_x.rows, order + 1, CV_32FC1);

	Mat copy;
	for (int i = 0; i <= order; i++)
	{
		copy = src_x.clone();
		pow(copy, i, copy);
		Mat M1 = X.col(i);
		copy.col(0).copyTo(M1);
	}
	Mat X_t, X_inv;
	transpose(X, X_t);
	Mat temp = X_t * X;
	Mat temp2;
	invert(temp, temp2);
	Mat temp3 = temp2 * X_t;
	Mat W = temp3 * src_y;

	W.copyTo(dst);

}

vector<float> LaneDetection::linspace(float start_in, float end_in, int num_in)
{
  vector<float> linspaced;

  float start = static_cast<float>(start_in);
  float end = static_cast<float>(end_in);
  float num = static_cast<float>(num_in);

  if (num == 0) { return linspaced; }
  if (num == 1) 
    {
      linspaced.push_back(start);
      return linspaced;
    }

  float delta = (end - start) / (num - 1);

  for(int i=0; i < num-1; ++i)
    {
      linspaced.push_back(start + delta * i);
    }
  linspaced.push_back(end); 

  return linspaced;
}

void LaneDetection::poly_fit_x(vector<float> const& ploty, vector<float>& fit_x, Mat const& line_fit)
{
	for (auto const& y : ploty) {
		float x = line_fit.at<float>(2, 0) * y * y + line_fit.at<float>(1, 0) * y + line_fit.at<float>(0, 0);
		fit_x.push_back(x);
	}

	return;
}

void LaneDetection::inverse_perspective(const Mat& warped_output, Mat& Minv, Mat& output_image)
{
  Mat color_warp = Mat::zeros(warped_output.size(), CV_8UC3);

  Minv = getPerspectiveTransform(warped_roi, original_roi);
  warpPerspective(warped_output, output_image, Minv, warped_output.size(),INTER_LINEAR);

}

void LaneDetection::get_inverse_points(vector<float>& plot_y, vector<float>& left_fit_x, vector<float>& right_fit_x, Mat& color_warp)
{
  vector<Point2f> pts_left;
  vector<Point2f>  pts_right;
  vector<Point> pts;

  for (int i = 0; i < plot_y.size(); i++) {
    pts_left.push_back(Point2f((float)left_fit_x[i], (float)plot_y[i])); 
    pts_right.push_back(Point2f((float)right_fit_x[i], (float)plot_y[i]));
  }

  pts.reserve(2 * plot_y.size());
  pts.insert(pts.end(), pts_left.begin(), pts_left.end());
  pts.insert(pts.end(), pts_right.rbegin(), pts_right.rend());

  vector<vector<Point>> ptsarray{ pts };

  fillPoly(color_warp, ptsarray, Scalar(0, 255, 0));
}

void LaneDetection::final_perspective(const Mat& color_warp, const Mat& original_image, Mat& Minv, Mat& output_image)
{
  Mat new_warp;

  warpPerspective(color_warp, new_warp, Minv, color_warp.size(),INTER_LINEAR);
  addWeighted(original_image, 1, new_warp, 0.3, 0, output_image);
}



void LaneDetection::init(string file_name, string output_file)
{
  capture.open(file_name);

  int frame_width = (int)capture.get(CAP_PROP_FRAME_WIDTH);
  int frame_height = (int)capture.get(CAP_PROP_FRAME_HEIGHT);
  
  video_output.open(output_file, VideoWriter::fourcc('A','V','C','1'), 25.0, Size(frame_width,frame_height),true);
  
  if(!capture.isOpened() )
      throw "Error when reading steam_mp4";

  if (!video_output.isOpened())
  {
      cout << "!!! Output video could not be opened" << endl;
      return;
  }


  vector<string> chessboard_images;
  cv::glob("/home/Olivera/LaneDetectionBCs/camera_cal/*.jpg", chessboard_images);

  bool success;
  int error;
  cv::Size board_size(8,6);
  Mat img = imread(chessboard_images[0]);
  cv::Size image_size(1280,720);

  success = calibrator.add_chessboard_points(chessboard_images, board_size);
  error = calibrator.calibration(image_size);

  if(capture.read(frame))
    trapezoid_roi();

}

bool LaneDetection::frame_processing()
{
  Mat color_filtered_image, filtered_image_gray;
  Mat binary_warped;
  Mat histogram;
  Point left_peak, right_peak;
  Mat new_left_fit, new_right_fit;
  Mat sliding_window_output;
  vector<float> plot_y, left_fit_x, right_fit_x;
  vector<Window> left_boxes, right_boxes;
  Mat color_warp = Mat::zeros(frame.size(), CV_8UC3);
  Mat inverse_perspective_output;
  Mat Minv(2,4,CV_32FC2);
  Mat output_frame;
  Mat undistorted_frame;
  int margin = 50;
  Mat sobel_output;

  undistorted_frame = calibrator.undistort_image(frame);
  frame = undistorted_frame;

  color_filter(color_filtered_image);
  cvtColor(color_filtered_image, filtered_image_gray, COLOR_RGB2GRAY);

  calculate_sobel(sobel_output);

  bitwise_and(filtered_image_gray, sobel_output, filtered_image_gray);

  perspective_transform(filtered_image_gray, binary_warped);

  get_histogram(binary_warped, histogram);

  calculate_lane_histogram(histogram, left_peak, right_peak);

  if(first_frame ==  true)
  {
    sliding_window(binary_warped, left_peak, right_peak, sliding_window_output, left_boxes, right_boxes);
    left_fit_lane = polyfit_windows(left_boxes);
    right_fit_lane = polyfit_windows(right_boxes);

    plot_y = linspace(0.0, (float)sliding_window_output.rows - 1, sliding_window_output.rows);

    poly_fit_x(plot_y, left_fit_x, left_fit_lane);
    poly_fit_x(plot_y, right_fit_x, right_fit_lane);

    first_frame  = false;

  } else 
  {
    non_sliding_window(binary_warped, left_fit_lane, right_fit_lane, new_left_fit, new_right_fit, margin);

    plot_y = linspace(0.0, (float)binary_warped.rows - 1, binary_warped.rows);

    poly_fit_x(plot_y, left_fit_x, new_left_fit);
    poly_fit_x(plot_y, right_fit_x, new_right_fit);
  
  }

  get_inverse_points(plot_y, left_fit_x, right_fit_x, color_warp);

  if(first_frame == true)
  {
    inverse_perspective(sliding_window_output, Minv, inverse_perspective_output);

  } else
  {
    inverse_perspective(binary_warped, Minv, inverse_perspective_output);
  }


  final_perspective(color_warp, frame, Minv, output_frame);

  video_output.write(output_frame);

  return capture.read(frame);
}

void LaneDetection::release()
{

  cout << "SAVED." << endl;
  capture.release();
	video_output.release();

}
