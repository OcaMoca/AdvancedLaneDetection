#include "../include/LaneDetection.hpp"

int main() {

    bool processed;

    string file_name = "test_videos/prestrojavanje.mp4";
    string output_file = "test_videos_output/prestrojavanje_test.mp4";

    LaneDetection ld;

    ld.init(file_name, output_file);

    int cnt = 0;

    while(true)
    {
        
        processed = ld.frame_processing();
        if(!processed)
            break;

        cnt++; 
        if(cnt == 200)
            break;
        cout << cnt << endl;

    }

    ld.release();

}
