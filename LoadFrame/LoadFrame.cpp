#include "LoadFrame.hpp"

LoadFrame::LoadFrame()
{
    ifstream config_file("configuration.json");
    j = nlohmann::json::parse(config_file);

    input_file = j.at("input_file").at("name_1");
    output_file = j.at("output_file").at("name_1");
}

void LoadFrame::open_input_video()
{
    capture.open(input_file);

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
    return capture.read(frame);
}

void LoadFrame::get_frame(Mat& curr_frame)
{
    curr_frame = this->frame;
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