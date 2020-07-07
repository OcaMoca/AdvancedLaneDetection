#include "TestModule.hpp"

TestModule::TestModule()
{}

struct compare {
    bool operator() (Point2f pt1, Point2f pt2) { return (pt1.y < pt2.y);}
} comp;

void TestModule::compare_parameters(vector<Point2f>& processed_points, vector<Point2i>& ground_truth_points)
{
    vector<Point2i>::reverse_iterator it1;
    vector<Point2f>::iterator it2;

    sort(ground_truth_points.rbegin(), ground_truth_points.rend(), comp);

    for(it1 = ground_truth_points.rbegin(), it2 = processed_points.begin(); it1 != ground_truth_points.rend(), it2 != processed_points.end(); it2++)
    {
        if((*it1).y == (int)((*it2).y))
        {
            //if((*it1).x == (int)((*it2).x) || ( (*it1).x + 15 >= (int)((*it2).x) && (*it1).x - 15 <= (int)((*it2).x) ))
            if((*it1).x == (int)((*it2).x))
            {
                cout << "Lane parameter coordinate (" << (int)(*it2).x << ", " << (int)(*it2).y << ") is in envrionment of ground truth point (" << (*it1).x << ", " <<  (*it1).y << ")." << endl;
            } 
            
            it1++;

        } else {
            continue;
        }

    }

    return;
}