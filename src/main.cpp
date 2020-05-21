#include "../include/LaneDetection.hpp"

int main() {

    bool processed;

    string file_name = "test_videos/project_video.mp4";
    string output_file = "test_videos_output/test2.mp4";

    LaneDetection ld;

    ld.init(file_name, output_file);

    int cnt = 0;

    while(true)
    {
        processed = ld.frame_processing();
        if(!processed)
            break;

        cnt++;  
        cout << cnt << endl;

    }

    ld.release();


}
