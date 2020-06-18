#include "../include/LoadFrame.hpp"

LoadFrame::LoadFrame()
{
    file_name = "/home/Olivera/LaneDetectionBSc/test_videos/project_video.mp4";
    output_file = "/home/Olivera/LaneDetectionBSc/test_videos_output/proba.mp4";
}

void LoadFrame::open_input_video()
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

}

bool LoadFrame::read_frame()
{
    if(capture.read(frame))
        return true;
    else 
        return false;
}

void LoadFrame::get_frame(Mat& curr_frame)
{
    curr_frame = frame;
}

void LoadFrame::write_to_output_video(Mat& output_frame)
{
    video_output.write(output_frame);
}

void LoadFrame::close_output_video()
{
    cout << "SAVED." << endl;
    capture.release();
	video_output.release();
}