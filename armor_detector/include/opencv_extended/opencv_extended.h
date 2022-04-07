#pragma once

#include <opencv2/opencv.hpp>

namespace cvex
{

    using namespace cv;
    using namespace std;

    RotatedRect& adjustRect(RotatedRect& rect)
    {
        float & angle = rect.angle;
        while (angle >= 90)
            angle -= 180.0;
        while (angle < -90)
            angle += 180.0;
        return rect;
    }

    template <typename T>
    float distance(const Point_<T>& point1, const Point_<T>& point2)
    {
        float dis = sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2));
        return dis;
    }

    void makeFrame(Mat * src_img, const vector<Point2f> img_vertex)
    {
        for (size_t i = 0; i < img_vertex.size(); i++)
        {
            if (i == 3)
            {
                line(*src_img, img_vertex[i], img_vertex[0], Scalar(0, 255, 0), 2);
                break;
            }
            line(*src_img, img_vertex[i], img_vertex[i + 1], Scalar(0, 255, 0), 2);

        }
    }

}