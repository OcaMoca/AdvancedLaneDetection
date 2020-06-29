#include "Window.hpp"

Window::Window(Mat& binary_warped, int x_center, int y_top, int width, int height, int min_pix)
{
    int margin;
    Mat window;

    this->x_center = x_center;
    this->y_top = y_top;
    this->width = width;
    this->height = height;
    this->min_pix = min_pix;

    margin = this->width;
    this->x_left = this->x_center - margin;
    this->y_bottom = y_top + this->height;
    this->x_right = this->x_center + margin;

    Rect rect(Point(x_left, y_bottom), Point(x_right, y_top));
    window = binary_warped(rect);
    findNonZero(window, non_zero);

    for(Point2f const& pt: non_zero)
    {
        non_zero_x.push_back(pt.x);
    }

}

Window Window::get_next_window(Mat& binary_warped)
{
    int cnt_nonzeros;
    cnt_nonzeros = count_nonzero();

    if(this->y_top <= 0) return Window();

    int new_y_top = y_top - height;
    int new_x_center = x_center;

    if (cnt_nonzeros >= min_pix)
    {
        new_x_center = (int)(accumulate( non_zero_x.begin(), non_zero_x.end(), 0.0) / cnt_nonzeros) + x_left;
    }

    if(new_x_center + this->width >= binary_warped.cols || new_x_center - this->width <= 0) return Window();

    Window result(binary_warped, new_x_center, new_y_top, this->width, this->height, this->min_pix);
    return result;
        
}


void Window::get_indices(Mat& x, Mat& y) const
{
	x.release();
	y.release();

	int npoints = count_nonzero();

	x = Mat::zeros(npoints, 1, CV_32FC1);
	y = Mat::zeros(npoints, 1, CV_32FC1);

	for (int i = 0; i < npoints; i++)
    {
		x.at<float>(i, 0) = non_zero[i].x + x_left;
		y.at<float>(i, 0) = non_zero[i].y + y_top;
	}

	return;
}


