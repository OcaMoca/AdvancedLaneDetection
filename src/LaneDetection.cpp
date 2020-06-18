#include "../include/LaneDetection.hpp"
using namespace std::chrono;


LaneDetection::LaneDetection()
{
  trap_bottom_width = 0.7f; 
  trap_top_width = 0.1f; 
  trap_height = 0.38f;
  car_hood = 50;

  /*trap_bottom_width = 0.28f; 
	trap_top_width = 0.05f; 
	trap_height = 0.3f;
	car_hood = 150;*/

  first_frame = true;
  result_optical = 0.0;
  wait_to_rearrange = 0;
  smoothed_angle = 0;

}

void LaneDetection::color_filter(Mat& frame, Mat& filtered_image)
{
    Mat mask1;
    Mat mask2;
    Mat image_hsv;
    Mat white_image;
    Mat yellow_image;

    cv::inRange(frame, Scalar(190,190,190), Scalar(255,255,255), mask1);
    //cv::inRange(frame, Scalar(80,80,80), Scalar(255,255,255), mask1);

    cvtColor(frame, image_hsv, COLOR_RGB2HSV);
    cv::inRange(image_hsv, Scalar(90,70,100), Scalar(110,255,255), mask2);

    cv::bitwise_and(frame, frame, white_image, mask1);
    
    cv::bitwise_and(frame, frame, yellow_image, mask2); 
    
    addWeighted(white_image, 1., yellow_image, 1., 0., filtered_image);

    return;
}

 void LaneDetection::calculate_sobel(Mat& frame,Mat& sobel_output)
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

void LaneDetection::trapezoid_roi(const Mat& frame)
{
    int width = (int)frame.size().width;
    int height = (int)frame.size().height;

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

void LaneDetection::perspective_transform(const Mat& filtered_image_gray, Mat& binary_warped)
{
  Mat binary_threshold;
  Mat M(2,4,CV_32FC2);

  M = getPerspectiveTransform(original_roi, warped_roi);

  binary_threshold =  Mat::zeros(filtered_image_gray.rows, filtered_image_gray.cols, CV_8UC3);
  threshold(filtered_image_gray, binary_threshold, 0, 255, THRESH_BINARY);

  warpPerspective(binary_threshold, binary_warped, M, filtered_image_gray.size(), INTER_LINEAR);

  return;
}

void LaneDetection::get_histogram(const Mat& binary_warped, Mat& histogram)
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

  return;
}

void LaneDetection::sliding_window(Mat& binary_warped, const Point& left_peak, const Point& right_peak, Mat& sliding_window_output, vector<Window>& left_boxes, vector<Window>& right_boxes)
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

  cvtColor(binary_warped, sliding_window_output, COLOR_GRAY2RGB);

  for(int i = 0; i < N_windows; i++)
  {
      rectangle(sliding_window_output, window_left.get_bottom_left_point(), window_left.get_top_right_point(), Scalar(0,255, 0), 2);
      rectangle(sliding_window_output, window_right.get_bottom_left_point(), window_right.get_top_right_point(), Scalar(0,255, 0), 2);

      left_boxes.push_back(window_left);
      right_boxes.push_back(window_right);
      
      window_left = window_left.get_next_window(gray_tmp);
      window_right = window_right.get_next_window(gray_tmp);
  }

  return;
}

Mat LaneDetection::polyfit_windows(const vector<Window>& windows)
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

void LaneDetection::calculate_lane_fit_next_frame(const vector<Point2f>& non_zero, Mat& lane_fit, vector<float>& xs, vector<float>& ys, int margin) 
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

void LaneDetection::non_sliding_window(const Mat& binary_warped, Mat& left_fit, Mat& right_fit, Mat& new_left_fit, Mat& new_right_fit, int margin)
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

  return;
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

void LaneDetection::poly_fit_x(const vector<float>& plot_y, vector<float>& fit_x, const Mat& line_fit)
{
	for (auto const& y : plot_y) {
		float x = line_fit.at<float>(2, 0) * y * y + line_fit.at<float>(1, 0) * y + line_fit.at<float>(0, 0);
		fit_x.push_back(x);
	}

	return;
}

void LaneDetection::original_perspective(const Mat& warped_output, Mat& Minv, Mat& original_perspective_output)
{
  Mat color_warp = Mat::zeros(warped_output.size(), CV_8UC3);

  Minv = getPerspectiveTransform(warped_roi, original_roi);
  warpPerspective(warped_output, original_perspective_output, Minv, warped_output.size(),INTER_LINEAR);

  return;
}

void LaneDetection::get_inverse_points(const vector<float>& plot_y, const vector<float>& left_fit_x, const vector<float>& right_fit_x, Mat& color_warp_output)
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

  fillPoly(color_warp_output, ptsarray, Scalar(0, 255, 0));

  return;
}

void LaneDetection::final_perspective(const Mat& color_warp, const Mat& original_image, Mat& Minv, Mat& final_perspective_output)
{
  Mat new_warp;

  warpPerspective(color_warp, new_warp, Minv, color_warp.size(),INTER_LINEAR);
  addWeighted(original_image, 1, new_warp, 0.3, 0, final_perspective_output);

  return;
}

float LaneDetection::calculate_curvature(const Mat& lane_fit, int height)
{
  float derivate_1, derivate_2;
  float R_curve;

  if(lane_fit.empty()) return 0;

  vector<float> ploty = linspace(0, height - 1, height);
  float y_eval = height - 1;
  
  float ym_per_pix = 30.0 / 720;
  float xm_per_pix = 3.7 / 700; 

  vector<float> xs;
  poly_fit_x(ploty, xs, lane_fit);

  Mat x(xs);
  Mat y(ploty);

  x.convertTo(x, CV_32F);
  y.convertTo(y, CV_32F);

  Mat poly_curvate = cv::Mat::zeros(3,1,CV_32F);

  polyfit(y * ym_per_pix, x * xm_per_pix, poly_curvate, 2);

  derivate_1 = 2 * poly_curvate.at<float>(2,0)  * y_eval * ym_per_pix + poly_curvate.at<float>(1,0); // f'(y) = 2Ay + B
  derivate_2 = 2 * poly_curvate.at<float>(2,0);  // f''(y) = 2A

  R_curve = pow((1 + pow(derivate_1, 2)), 1.5) / derivate_2; //R_curve = (1 + (2Ay + B)^2)^3/2 / 2A

  if(R_curve > 1000 || R_curve < -1000)
    R_curve = pow((1 + pow(derivate_1, 2)), 1.5) / abs(derivate_2); //R_curve = (1 + (2Ay + B)^2)^3/2 / |2A|*/

  return R_curve;

}

float LaneDetection::trim_mean(vector<float>& array, float pertentage)
{
  float average;
  vector<float> array_tmp = array;
  std::sort(array_tmp.begin(), array_tmp.end());

  int num_to_remove = (array_tmp.size() * pertentage) / 2;

  array_tmp.begin() = array_tmp.begin() + num_to_remove;
  array_tmp.end() = array_tmp.end() - num_to_remove;

  array_tmp.resize(array_tmp.size() - 2*num_to_remove);

  average = accumulate( array_tmp.begin(), array_tmp.end(), 0.0) / array_tmp.size();
  
  return average;

}


void LaneDetection::convert_to_optical(const Mat& curr_frame, Mat& convert_to_optical_output)
{
  Mat prev_frame_gray, curr_frame_gray;

  vector<uchar> status;
  vector<float> error;

  float x1_meter; 
  float x2_meter; 
  float y1_meter; 
  float y2_meter;

  vector<float> distance;
  float time = 1 / 60.0;
  vector<float> speed;

  float speed_avg;
  float distance_avg;

  vector<Point2f> new_point_vector, good_new;

  float ym_per_pix = 30.0 / 720; //30.0 / 720
  float xm_per_pix = 3.7 / 700;  //3.7 / 700

    vector<Scalar> colors;
    RNG rng;
    for(int i = 0; i < 100; i++)
    {
        int r = rng.uniform(0, 256);
        int g = rng.uniform(0, 256);
        int b = rng.uniform(0, 256);
        colors.push_back(Scalar(r,g,b));
    }

  cvtColor(prev_frame, prev_frame_gray, COLOR_BGR2GRAY);
  cvtColor(curr_frame, curr_frame_gray, COLOR_BGR2GRAY);

  goodFeaturesToTrack(prev_frame_gray, old_point_vector, 300, 0.3, 7, Mat(), 7, false, 0.04);

  Mat mask = Mat::zeros(prev_frame.size(), prev_frame.type());
  
  TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 10, 0.03);

  calcOpticalFlowPyrLK(prev_frame_gray, curr_frame_gray, old_point_vector, new_point_vector, status, error, Size(15, 15), 2, criteria);

  for(int i = 0; i < old_point_vector.size(); i++)
  {
    if(status[i] == 1)
    {
    
      if((new_point_vector[i] - old_point_vector[i]).x > 5 || (new_point_vector[i] - old_point_vector[i]).x < -5 || (new_point_vector[i] - old_point_vector[i]).y < 5)
        continue;

      good_new.push_back(new_point_vector[i]);
      line(mask, new_point_vector[i], old_point_vector[i], colors[i], 2);
      circle(curr_frame, new_point_vector[i], 5, colors[i], -1);

      x1_meter = xm_per_pix * new_point_vector[i].x;
      x2_meter = xm_per_pix * old_point_vector[i].x;

      y1_meter = ym_per_pix * new_point_vector[i].y;
      y2_meter = ym_per_pix * old_point_vector[i].y;

      distance.push_back(sqrt(pow(x1_meter - x2_meter, 2) + pow(y1_meter - y2_meter, 2)));
    }

  }

  if(distance.size() == 0)
  {
    return;
  }

  distance_avg = trim_mean(distance, 0.2);

  speed_avg = distance_avg / time * 3.6;
  succ_avg_speed.push_back(speed_avg);

  if(succ_avg_speed.size() == 50)
  {
    result_optical = (int)trim_mean(succ_avg_speed, 0.2);
    succ_avg_speed.erase(succ_avg_speed.begin());
  } 
 
  add(curr_frame, mask, convert_to_optical_output);
  old_point_vector = good_new;

  return;
}


float LaneDetection::calculate_car_offset(Mat& undistorted, Mat& left_fit, Mat& right_fit)
{
  float bottom_y = undistorted.rows - 1;
  float bottom_x_left = left_fit.at<float>(2, 0) * bottom_y * bottom_y + left_fit.at<float>(1, 0) * bottom_y + left_fit.at<float>(0, 0);
  float bottom_x_right = right_fit.at<float>(2, 0) * bottom_y * bottom_y + right_fit.at<float>(1, 0) * bottom_y + right_fit.at<float>(0, 0);

  float car_offset = undistorted.cols / 2 - 100 - (bottom_x_left + bottom_x_right) / 2;

  float xm_per_pix = 3.7 / 700;
  car_offset = car_offset * xm_per_pix;

  return car_offset;

}

void LaneDetection::steering_wheel_rotation(float R_curve_avg, Mat& steering_wheel_rotated)
{
  float degree_of_curve;
  Mat M;

  degree_of_curve = -5729.57795 / R_curve_avg;
  smoothed_angle = smoothed_angle + 0.2 * pow(abs(degree_of_curve - smoothed_angle), 2.0/3.0) * (degree_of_curve - smoothed_angle) / abs(degree_of_curve - smoothed_angle);
  M = getRotationMatrix2D(Point2f(steering_wheel.cols / 2.0, steering_wheel.rows / 2.0), smoothed_angle , 1);
  warpAffine(steering_wheel, steering_wheel_rotated, M, Size(steering_wheel.cols, steering_wheel.rows));

  return;

}

void LaneDetection::init()
{
  vector<string> chessboard_images;
  cv::glob("/home/Olivera/LaneDetectionBSc/camera_cal/*.jpg", chessboard_images);

  bool success;
  int error;
  cv::Size board_size(8,6);
  Mat img = imread(chessboard_images[0]);
  cv::Size image_size(1280,720);
   

  success = calibrator.add_chessboard_points(chessboard_images, board_size);
  error = calibrator.calibration(image_size);

  steering_wheel = imread("/home/Olivera/LaneDetectionBSc/test_images/steering_wheel_new.png", IMREAD_UNCHANGED);

  return;
}


void LaneDetection::frame_processing(Mat& frame, Mat& output_frame)
{
  Mat color_filtered_image, filtered_image_gray;
  Mat binary_warped;
  Mat histogram;
  Point left_peak(0,0), right_peak(0,0);
  Mat new_left_fit, new_right_fit;
  Mat sliding_window_output;
  vector<float> plot_y, left_fit_x, right_fit_x;
  vector<Window> left_boxes, right_boxes;
  Mat color_warp = Mat::zeros(frame.size(), CV_8UC3);;
  Mat inverse_perspective_output;
  Mat Minv(2,4,CV_32FC2);
  //Mat output_frame;
  Mat undistorted_frame;
  int margin = 50;
  Mat sobel_output;
  float R_curve_left, R_curve_right, R_curve_avg;
  Mat optical_out;
  int speed;
  Mat steering_wheel_rotated; 
  Mat result;
  
  calibrator.undistort_image(frame, undistorted_frame);

  color_filter(undistorted_frame, color_filtered_image);
  cvtColor(color_filtered_image, filtered_image_gray, COLOR_RGB2GRAY);

  calculate_sobel(undistorted_frame, sobel_output);

  bitwise_and(filtered_image_gray, sobel_output, filtered_image_gray);

  perspective_transform(filtered_image_gray, binary_warped);

  get_histogram(binary_warped, histogram);

  calculate_lane_histogram(histogram, left_peak, right_peak);

  
  /*if(wait_to_rearrange == 1){
    if(left_peak.x > (binary_warped.cols / 2) - 100 || left_peak.x < 300 || right_peak.x < (binary_warped.cols / 2) + 100 || right_peak.x > binary_warped.cols - 300)
    {
      video_output.write(undistorted_frame);
      return capture.read(frame);
    } else {
      first_frame = true;
      wait_to_rearrange = 0;
    }

  }*/
  
  if(first_frame ==  true)
  {
    sliding_window(binary_warped, left_peak, right_peak, sliding_window_output, left_boxes, right_boxes);
    
    left_fit_lane = polyfit_windows(left_boxes);
    right_fit_lane = polyfit_windows(right_boxes);

    plot_y = linspace(0.0, (float)sliding_window_output.rows - 1, sliding_window_output.rows);

    poly_fit_x(plot_y, left_fit_x, left_fit_lane);
    poly_fit_x(plot_y, right_fit_x, right_fit_lane);

    R_curve_left = calculate_curvature(left_fit_lane, undistorted_frame.rows);
    R_curve_right = calculate_curvature(right_fit_lane, undistorted_frame.rows);

    R_curve_avg = (R_curve_left + R_curve_right) / 2;

    prev_frame = sliding_window_output;
   

  } else {
    non_sliding_window(binary_warped, left_fit_lane, right_fit_lane, new_left_fit, new_right_fit, margin);
    
    plot_y = linspace(0.0, (float)binary_warped.rows - 1, binary_warped.rows);

    poly_fit_x(plot_y, left_fit_x, new_left_fit);
    poly_fit_x(plot_y, right_fit_x, new_right_fit);

    R_curve_left = calculate_curvature(new_left_fit, undistorted_frame.rows);
    R_curve_right = calculate_curvature(new_right_fit, undistorted_frame.rows);

    R_curve_avg = (R_curve_left + R_curve_right) / 2;

    cvtColor(binary_warped, binary_warped, COLOR_GRAY2BGR);

    convert_to_optical(binary_warped, optical_out);

    prev_frame = binary_warped;

    

  }

  get_inverse_points(plot_y, left_fit_x, right_fit_x, color_warp);

  if(first_frame == true)
  {
    original_perspective(sliding_window_output, Minv, inverse_perspective_output);
    first_frame  = false;

  } else
  {
    original_perspective(binary_warped, Minv, inverse_perspective_output);
  }

  final_perspective(color_warp, undistorted_frame, Minv, output_frame);

  float car_offset;
  car_offset = calculate_car_offset(undistorted_frame, left_fit_lane, right_fit_lane);

   steering_wheel_rotation(R_curve_avg, steering_wheel_rotated);

  char radius_text[100];
  char speed_text[100];
  char car_offset_text[100];
  sprintf(radius_text, "Radius of curvature: %f m", R_curve_avg);
  sprintf(speed_text, "Car speed: %d km/h", result_optical);
  sprintf(car_offset_text, "Car offset: %f m", car_offset);

  cv::putText(output_frame, radius_text, Point2f(50,50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,0,0));

  if(result_optical > 120)
  {
    cv::putText(output_frame, speed_text, Point2f(50,100), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,255,0));
  } else {
    cv::putText(output_frame, speed_text, Point2f(50,100), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,0,0));
  }

  cv::putText(output_frame, car_offset_text, Point2f(50,150), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,0,0));

  Rect roi(Point(output_frame.cols - steering_wheel_rotated.cols, output_frame.rows - steering_wheel_rotated.rows), Size(steering_wheel_rotated.cols, steering_wheel_rotated.rows));
  cvtColor(steering_wheel_rotated, steering_wheel_rotated, COLOR_BGRA2BGR);
  steering_wheel_rotated.copyTo(output_frame(roi));

  //video_output.write(output_frame);

  /*if(car_offset >= 1)
  {
    wait_to_rearrange = 1;
  }*/

  //return capture.read(frame);

  return;
}
