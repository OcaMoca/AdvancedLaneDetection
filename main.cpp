#include "LoadFrame/LoadFrame.hpp"
#include "CameraCalibration/CameraCalibration.hpp"
#include "FrameProcessing/FrameProcessing.hpp"
#include "FeatureExtraction/FeatureExtraction.hpp"
#include "HMI/HMI.hpp"

#include <nlohmann/json.hpp>
using json = nlohmann::json;

Mat frame;
Mat output_frame;
Mat prev_frame;
bool first_frame = true;
Mat steering_wheel;
Mat left_fit_lane, right_fit_lane;


int main()
{
    json j; 
    
    int cnt = 0;
    float smoothed_angle = 0.0;
    
    ifstream config_file("configuration.json");
    j = nlohmann::json::parse(config_file);
    
    LoadFrame load_frame;
    CameraCalibration calibrator;
    FrameProcessing frame_processor;
    FeatureExtraction feature_extractor;
    HMI hmi;

    string steering_wheel_image = j.at("steering_wheel").at("name");
    steering_wheel = imread(steering_wheel_image, IMREAD_UNCHANGED);

    /*LOADING FIRST FRAME*/

    load_frame.open_input_video();
    load_frame.read_frame();
    load_frame.get_frame(frame);

    output_frame = Mat::zeros(frame.rows, frame.cols, CV_32FC2);

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
        Mat undistorted_frame;
        Mat color_filtered_image, filtered_image_gray;
        Mat sobel_output;
        Mat binary_warped;

        Mat histogram;
        Point left_peak(0,0), right_peak(0,0);
        vector<Window> left_boxes, right_boxes;
        Mat sliding_window_output;
        Mat optical_flow_output;
        Mat new_left_fit, new_right_fit;
        vector<float> plot_y, left_fit_x, right_fit_x;
        float R_curve_left, R_curve_right, R_curve_avg;
        int car_speed;
        float car_offset;
        Mat steering_wheel_rotated;
        Mat color_warp_output = Mat::zeros(frame.size(), CV_8UC3); 

        calibrator.undistort_image(frame, undistorted_frame);

        frame_processor.color_filter(undistorted_frame, color_filtered_image, white_range_s);
        cvtColor(color_filtered_image, filtered_image_gray, COLOR_RGB2GRAY);

        frame_processor.calculate_sobel(undistorted_frame, sobel_output);

        bitwise_and(filtered_image_gray, sobel_output, filtered_image_gray);

        frame_processor.perspective_transform(filtered_image_gray, binary_warped);

        /*FEATURE EXTRACTION*/
        
        plot_y = feature_extractor.linspace(0.0, (float)binary_warped.rows - 1, binary_warped.rows);

        if(first_frame)
        {
            feature_extractor.get_histogram(binary_warped, histogram);

            feature_extractor.calculate_lane_histogram(histogram, left_peak, right_peak);

            feature_extractor.sliding_window(binary_warped, left_peak, right_peak, sliding_window_output, left_boxes, right_boxes);

            left_fit_lane = feature_extractor.polyfit_windows(left_boxes);
            right_fit_lane = feature_extractor.polyfit_windows(right_boxes);

            feature_extractor.poly_fit_x(plot_y, left_fit_x, left_fit_lane);
            feature_extractor.poly_fit_x(plot_y, right_fit_x, right_fit_lane);

            R_curve_left = feature_extractor.calculate_curvature(left_fit_lane, binary_warped.rows);
            R_curve_right = feature_extractor.calculate_curvature(right_fit_lane, binary_warped.rows);
            feature_extractor.calculate_car_offset(undistorted_frame, left_fit_lane, right_fit_lane);

            cvtColor(sliding_window_output, sliding_window_output, COLOR_BGR2GRAY);
            prev_frame = sliding_window_output;

        } else {
            feature_extractor.non_sliding_window(binary_warped, left_fit_lane, right_fit_lane, new_left_fit, new_right_fit, 50);

            feature_extractor.poly_fit_x(plot_y, left_fit_x, new_left_fit);
            feature_extractor.poly_fit_x(plot_y, right_fit_x, new_right_fit);

            R_curve_left = feature_extractor.calculate_curvature(new_left_fit, binary_warped.rows);
            R_curve_right = feature_extractor.calculate_curvature(new_right_fit, binary_warped.rows);

            prev_frame = binary_warped;
            
            feature_extractor.convert_to_optical(binary_warped, prev_frame, optical_flow_output);
            car_speed = feature_extractor.get_car_speed();

            feature_extractor.calculate_car_offset(undistorted_frame, new_left_fit, new_right_fit);
            
        }

        R_curve_avg = (R_curve_left + R_curve_right) / 2;
        
        car_offset = feature_extractor.get_car_offset();
        feature_extractor.steering_wheel_rotation(steering_wheel, R_curve_avg, smoothed_angle, steering_wheel_rotated);

        /*HMI*/

        hmi.get_inverse_points(plot_y, left_fit_x, right_fit_x, color_warp_output);

        hmi.final_perspective(color_warp_output, undistorted_frame, original_roi, warped_roi, output_frame);

        hmi.show_values(R_curve_avg, car_offset, car_speed, output_frame);
        hmi.show_steering_wheel(output_frame, steering_wheel_rotated, 50);

        load_frame.write_to_output_video(output_frame);

        if(load_frame.read_frame())
            load_frame.get_frame(frame);
        else
            break;

        cnt++; 

        cout << cnt << endl;

    }

    load_frame.close_output_video();

}
