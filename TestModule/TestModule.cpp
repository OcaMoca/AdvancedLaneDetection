#include "TestModule.hpp"

TestModule::TestModule()
{}

struct compare {
    bool operator() (Point2f pt1, Point2f pt2) { return (pt1.y < pt2.y);}
} comp;

void TestModule::compare_parameters(vector<Point2f>& processed_points, vector<Point2i>& ground_truth_points, const string line_position)
{
    vector<Point2i>::reverse_iterator it1;
    vector<Point2f>::iterator it2;

    sort(ground_truth_points.rbegin(), ground_truth_points.rend(), comp);

    for(it1 = ground_truth_points.rbegin(), it2 = processed_points.begin(); it1 != ground_truth_points.rend(), it2 != processed_points.end(); it2++)
    {
        if((*it1).y == (int)((*it2).y))
        {
            if( ( (640 >= (*it1).x) && (640 >= (*it2).x) && line_position == "left") || ((640 < (*it1).x) && (640 < (*it2).x) && line_position == "right") )
            {

                error.push_back(abs((*it1).x - (int)((*it2).x))); // store absolute value of differece between x coordinates of processed and ground truth point
            } 
            
            it1++;

        } else {
            continue;
        }

    }

    return;
}

void TestModule::calculate_params_error(vector<Point2i>& test_points, vector<Point2f>& pts_left, vector<Point2f>& pts_right)
{
    ofstream error_values;
    error_values.open("/home/Olivera/LaneDetectionBSc/error_values1.txt", _S_app); 
    std::ostream_iterator<int> error_values_it(error_values," ");

    /* Left line */
    compare_parameters(pts_left, test_points, "left");
    copy(error.begin(), error.end(), error_values_it);
    error.clear();

    /* Right line */
    compare_parameters(pts_right, test_points, "right");
    copy(error.begin(), error.end(), error_values_it);
    error.clear();

    test_points.clear();
    error_values << "\n";
    error_values.close();
}

void get_frame_params(int& frame_num, vector<Point2f>& pts_left, vector<Point2f>& pts_right, vector<Point2f>& all_pts)
{


}

void TestModule::calculate_specific_point_error(vector<Point2i>& test_points, vector<Point2f>& pts_left, vector<Point2f>& pts_right)
{

    vector<Point2i>::reverse_iterator it1;
    vector<Point2f>::iterator it2;

    /*sort(ground_truth_points.rbegin(), ground_truth_points.rend(), comp);

    for(it1 = ground_truth_points.rbegin(), it2 = processed_points.begin(); it1 != ground_truth_points.rend(), it2 != processed_points.end(); it2++)
    {

    }*/


}


        

