#include "../include/LaneDetection.hpp"
#include "../include/LoadFrame.hpp"


int main() {

    bool processed;
    Mat output_frame;
    Mat curr_frame;

    LaneDetection ld;
    LoadFrame lf;

    lf.open_input_video();
    lf.read_frame();
    lf.get_frame(curr_frame);
    output_frame = Mat::zeros(curr_frame.rows, curr_frame.cols, CV_32FC2);

    ld.init();
    ld.trapezoid_roi(curr_frame);

    int cnt = 0;

    while(true)
    {
       
        ld.frame_processing(curr_frame, output_frame);
    
        lf.write_to_output_video(output_frame);

        if(lf.read_frame())
            lf.get_frame(curr_frame);
        else
            break;

        cnt++; 

        cout << cnt << endl;

    }

    lf.close_output_video();

}

/* 
auto start = high_resolution_clock::now();
auto stop = high_resolution_clock::now();

auto duration = duration_cast<milliseconds> (stop - start);
cout << duration.count() << endl; */
