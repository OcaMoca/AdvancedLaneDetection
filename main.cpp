#include "LoadFrame/LoadFrame.hpp"
#include "CameraCalibration/CameraCalibration.hpp"
#include "FrameProcessing/FrameProcessing.hpp"
#include "FeatureExtraction/FeatureExtraction.hpp"
#include "HMI/HMI.hpp"
#include "TestModule/TestModule.hpp"

#include <nlohmann/json.hpp>
using json = nlohmann::json;

vector<Point2i> test_points;
ofstream ground_truth;
ofstream points_base;

void CallBackFunc(int event, int x, int y, int flags, void* param)
{  
    if  ( event == EVENT_LBUTTONDOWN )
    {
        char test_text_x[50], test_text_y[50]; 
        sprintf(test_text_x, "Position ( %d,", x);
        sprintf(test_text_y, "%d )", y);
        strcat(test_text_x, test_text_y);
        
        ground_truth << strcat(test_text_x, "\n");
        
        test_points.push_back(Point2i(x,y)); 
    }
}

int main()
{
    json j; 

    Mat frame;
    Mat output_frame;
    Mat prev_frame;
    int cnt = 0;

    float smoothed_angle = 0.0;
    bool first_frame = true;
    Mat steering_wheel;
    Mat left_fit_lane, right_fit_lane;
    int wait_to_rearrange = 0;
    
    string input_file_s;
    string output_file_s;
    
    ifstream config_file("configuration.json");
    j = nlohmann::json::parse(config_file);

    input_file_s = j.at("input_file").at("name_1");
    output_file_s = j.at("output_file").at("name_1");

    LoadFrame load_frame(input_file_s, output_file_s);
    CameraCalibration calibrator;
    FrameProcessing frame_processor;
    FeatureExtraction feature_extractor;
    HMI hmi;
    TestModule test;

    string steering_wheel_image = j.at("steering_wheel").at("name");
    steering_wheel = imread(steering_wheel_image, IMREAD_UNCHANGED);
    output_frame = Mat::zeros(frame.rows, frame.cols, CV_32FC2);
    prev_frame = Mat::zeros(frame.rows, frame.cols, CV_32FC2);

    /*LOADING FIRST FRAME*/

    load_frame.open_input_video();
    load_frame.read_frame();
    load_frame.get_frame(frame);

    /*CAMERA CALIBRATION*/

    vector<string> chessboard_images;
    string calibration_images = j.at("calibration_images").at("name");
    cv::glob(calibration_images, chessboard_images);

    bool success;
    int error;
    cv::Size board_size(8,6);
    Mat img = imread(chessboard_images[0]);
    cv::Size image_size(1280,720);
    
    success = calibrator.add_chessboard_points(chessboard_images, board_size);
    error = calibrator.calibration(image_size);

    /*FRAME PROCESSING*/
    
    vector<Point2f> original_roi, warped_roi;

    Mat undistorted_frame;
    Mat color_filtered_image, filtered_image_gray;
    Mat sobel_output;
    Mat binary_warped;
    Mat histogram;
    Point left_peak(0,0), right_peak(0,0);
    vector<float> plot_y;
    Mat sliding_window_output;
    vector<Window> left_boxes, right_boxes;
    Mat optical_flow_output;
    Mat new_left_fit, new_right_fit;
    float R_curve_left, R_curve_right, R_curve_avg;
    int car_speed;
    float car_offset;
    Mat steering_wheel_rotated;

    /*variables init*/
    undistorted_frame = Mat::zeros(frame.rows, frame.cols, CV_32FC2);
    color_filtered_image = Mat::zeros(frame.rows, frame.cols, CV_32FC2);
    filtered_image_gray = Mat::zeros(frame.rows, frame.cols, CV_32FC2);
    sobel_output = Mat::zeros(frame.rows, frame.cols, CV_32FC2);
    binary_warped = Mat::zeros(frame.rows, frame.cols, CV_32FC2);
    sliding_window_output = Mat::zeros(1, frame.cols, CV_32FC2);
    optical_flow_output = Mat::zeros(1, frame.cols, CV_32FC2);
    steering_wheel_rotated = steering_wheel;
    car_speed = 0;
    car_offset = 0.0;
    R_curve_left = 0.0;
    R_curve_right = 0.0;
    R_curve_avg = 0.0;

    string trap_bottom_width_s = j.at("trap_bottom_width").at("value_1");
    string trap_top_width_s = j.at("trap_top_width").at("value_1");
    string trap_height_s = j.at("trap_height").at("value_1");
    string car_hood_s = j.at("car_hood").at("value_1");
    string white_range_s = j.at("white_range").at("value_1");

    frame_processor.trapezoid_roi(frame, trap_bottom_width_s, trap_top_width_s, trap_height_s, car_hood_s);

    original_roi = frame_processor.get_original_roi();
    warped_roi = frame_processor.get_warped_roi();

    while(true)
    {
        Mat color_warp_output = Mat::zeros(frame.size(), CV_8UC3); 
        vector<float> left_fit_x, right_fit_x;
        vector<Point2f> pts_left, pts_right;
        vector<Point> pts;
        histogram = Mat::zeros(1, frame.cols, CV_32FC2);
        
        calibrator.undistort_image(frame, undistorted_frame);

        frame_processor.color_filter(undistorted_frame, color_filtered_image, white_range_s);
        cvtColor(color_filtered_image, filtered_image_gray, COLOR_RGB2GRAY);

        frame_processor.calculate_sobel(undistorted_frame, sobel_output);

        bitwise_and(filtered_image_gray, sobel_output, filtered_image_gray);

        frame_processor.perspective_transform(filtered_image_gray, binary_warped);

        /*FEATURE EXTRACTION*/
        
        plot_y = feature_extractor.linspace(0.0, (float)binary_warped.rows - 1, (float)binary_warped.rows);

        feature_extractor.get_histogram(binary_warped, histogram);

        feature_extractor.calculate_lane_histogram(histogram, left_peak, right_peak);

        if(wait_to_rearrange == 1)
        {
            if(left_peak.x > (binary_warped.cols / 2) - 100 || left_peak.x < 300 || right_peak.x < (binary_warped.cols / 2) + 100 || right_peak.x > binary_warped.cols - 300)
            {
                load_frame.write_to_output_video(undistorted_frame);
                if(load_frame.read_frame())
                {   
                    load_frame.get_frame(frame);
                    continue;
                } 
    
            } else {
                first_frame = true;
                wait_to_rearrange = 0;
            }
        }

        if(first_frame)
        {
            feature_extractor.sliding_window(binary_warped, left_peak, right_peak, sliding_window_output, left_boxes, right_boxes);

            left_fit_lane = feature_extractor.polyfit_windows(left_boxes);
            right_fit_lane = feature_extractor.polyfit_windows(right_boxes);

            feature_extractor.poly_fit_x(plot_y, left_fit_x, left_fit_lane);
            feature_extractor.poly_fit_x(plot_y, right_fit_x, right_fit_lane);

            R_curve_left = feature_extractor.calculate_curvature(left_fit_lane, binary_warped.rows);
            R_curve_right = feature_extractor.calculate_curvature(right_fit_lane, binary_warped.rows);

            feature_extractor.calculate_car_offset(undistorted_frame, left_fit_lane, right_fit_lane);

            cvtColor(binary_warped, binary_warped, COLOR_GRAY2BGR);
            prev_frame = binary_warped;
            first_frame = false;

        } else {
            feature_extractor.non_sliding_window(binary_warped, left_fit_lane, right_fit_lane, new_left_fit, new_right_fit, 50);

            feature_extractor.poly_fit_x(plot_y, left_fit_x, new_left_fit);
            feature_extractor.poly_fit_x(plot_y, right_fit_x, new_right_fit);

            R_curve_left = feature_extractor.calculate_curvature(new_left_fit, binary_warped.rows);
            R_curve_right = feature_extractor.calculate_curvature(new_right_fit, binary_warped.rows);
            
            cvtColor(binary_warped, binary_warped, COLOR_GRAY2BGR);
            feature_extractor.convert_to_optical(binary_warped, prev_frame, optical_flow_output);
        
            feature_extractor.calculate_car_offset(undistorted_frame, new_left_fit, new_right_fit);

            prev_frame = binary_warped;
            
        }

        R_curve_avg = (R_curve_left + R_curve_right) / 2;
        car_offset = feature_extractor.get_car_offset();
        car_speed = feature_extractor.get_car_speed();
        
        feature_extractor.steering_wheel_rotation(steering_wheel, R_curve_avg, smoothed_angle, steering_wheel_rotated);

        /*HMI*/

        hmi.get_inverse_points(plot_y, left_fit_x, right_fit_x, color_warp_output, pts_left, pts_right, pts);

        hmi.final_perspective(color_warp_output, undistorted_frame, original_roi, warped_roi, output_frame);

        hmi.show_values(R_curve_avg, car_offset, car_speed, output_frame);
        hmi.show_steering_wheel(output_frame, steering_wheel_rotated, 50);

        load_frame.write_to_output_video(output_frame);

        if(car_offset >= 1.5)
        {
            wait_to_rearrange = 1;
        }

        if(load_frame.read_frame())
            load_frame.get_frame(frame);
        else
            break;

        cnt++;

        cout << cnt << endl;

        /*Make base of points for first 15 frames*/

        Mat test_frame;
        Mat M(2,4,CV_32FC2);

        M = getPerspectiveTransform(original_roi, warped_roi);
        warpPerspective(undistorted_frame, test_frame, M, undistorted_frame.size(), INTER_LINEAR);
        line(test_frame, Point(0, 620), Point(test_frame.cols, 620), Scalar(255,0,0), 3);

        if(cnt <= 15)
        {
            ground_truth.open("/home/Olivera/LaneDetectionBSc/ground_truth3.txt", _S_app);
            ground_truth << "FRAME " << cnt << "\n";
            namedWindow("Test", 1);
            setMouseCallback("Test", CallBackFunc, NULL);
            imshow("Test", test_frame);
            waitKey(0);
            ground_truth.close();

            points_base.open("/home/Olivera/LaneDetectionBSc/points_base3.txt", _S_app);
            points_base << "*************** FRAME" << cnt << "***************\n";
            std::ostream_iterator<Point> points_base_it(points_base," ");
            copy(pts.begin(), pts.end(), points_base_it);
            points_base << "\n\n";
            points_base.close();

        }


        if(cnt != 1 && cnt != 1000 && cnt != 500)
        {
            continue;
        }

        test.calculate_params_error(test_points, pts_left, pts_right); 

    }

    load_frame.close_output_video();
 
   

    

}
