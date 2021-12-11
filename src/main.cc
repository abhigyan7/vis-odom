#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include <stdint.h>
#include "triangulate.h"

//TODO create constructors
//TODO build pose graph from triangulated data
//TODO encapsulate triangulation state into a class

typedef Eigen::Vector3f WorldPoint;
typedef Eigen::Vector2f ImagePoint;

class Frame {
    public:
        cv::Mat image;
        // maps point id (WorldPoint.id) to its projection in this frame
        std::unordered_map<int, ImagePoint> projected_points;
        Eigen::Matrix3f R;
        Eigen::Vector3f t;
};

class WorldMap {
    private:
        std::vector<WorldPoint> world_points;
        std::vector<Frame> frames;
        cv::Mat img_1, img_2;
        std::vector<cv::Point2f> points_1;
        std::vector<cv::Point2f> points_2;
        double focal;
        cv::Point2f pp;

    public:
    WorldMap (double focal, cv::Point2f pp)
    {
        this->focal = focal;
        this->pp = pp;
    }

    bool register_new_image(cv::Mat &new_img)
    {

        if (this->frames.size() == 0)
        {
            Frame frame_1;
            frame_1.image = new_img;
            this->frames.push_back(frame_1);
            this->img_1 = new_img;

            return true;
        }
        Frame frame_2;
        frame_2.image = new_img;
        this->frames.push_back(frame_2);
        this->img_2 = new_img;
        this->img_2 = new_img;
        auto orb_detector = cv::ORB::create();

        auto feature_matcher = cv::BFMatcher::create();

        std::vector<uchar> status;
        std::vector<float> error;

        std::vector<cv::KeyPoint> keypoints_1;
        std::vector<cv::KeyPoint> keypoints_2;
        cv::Mat descriptors_1;
        cv::Mat descriptors_2;
        orb_detector->detectAndCompute(this->img_2, cv::Mat(), keypoints_1, descriptors_1);
        orb_detector->detectAndCompute(this->img_2, cv::Mat(), keypoints_2, descriptors_2);

        std::vector<cv::DMatch> matches;
        feature_matcher->match(descriptors_1, descriptors_2, matches);

        std::vector<cv::Point2f> points_1, points_2;
        std::vector<cv::Point2f> temp_points_1, temp_points_2;
        cv::KeyPoint::convert(keypoints_1, temp_points_1);
        cv::KeyPoint::convert(keypoints_2, temp_points_2);

        for (auto match : matches)
        {
            points_1.push_back(temp_points_1[match.queryIdx]);
            points_2.push_back(temp_points_2[match.trainIdx]);
            1;
        }

        std::cout << "Pushed frame: " << this->frames.size() << std::endl;
        this->img_1 = img_2;

        //std::vector<cv::Point2f> points_2;
        //std::cout << points_1.size() << std::endl;

        //size_t i, k;
        //for (i = k = 0; i < this->points_1.size(); i++)
        //{
        //    if (! status[i])
        //        continue;
        //    this->points_1[k++] = this->points_1[i];
        //    this->points_2[k  ] = this->points_2[i];
        //}

        //this->points_1.resize(k);
        //this->points_2.resize(k);

        return true;

    }
};



int main(int argc, char** argv )
{
    if ( argc != 2 )
    {
        printf("usage: main <video-file>.mp4\n");
        return -1;
    }
    cv::VideoCapture vidcap;
    vidcap.open(argv[1]);
    //polyscope::init();
    std::vector<glm::vec3> points;

    cv::Point2f pp;
    pp.x = 0.0;
    pp.y = 0.0;

    WorldMap map(700.0, pp);
    cv::Mat image, image_c;

    bool has_new_frames = true;
    has_new_frames = vidcap.read(image_c);
    while (has_new_frames)
    {
        cv::cvtColor(image_c, image, cv::COLOR_BGR2GRAY);
        map.register_new_image(image);
        has_new_frames = vidcap.read(image_c);
    }

    cv::Mat world_points;
    cv::Mat R, t;
    //triangulate_points(image_1, image_2, points_1, 700.0, pp, R, t, world_points);
    // visualize!
    //
    return 0;
    //
    std::vector<glm::vec3> world_glm;

    for (int i = 0; i < world_points.rows; i++)
    {
        world_glm.push_back(
            glm::vec3(
                world_points.at<float>(i, 0),
                world_points.at<float>(i, 1),
                world_points.at<float>(i, 2)
            )
        );
    }

    polyscope::PointCloud* psCloud = polyscope::registerPointCloud("really great points", world_glm);

    // set some options
    psCloud->setPointRadius(0.002);
    psCloud->setPointRenderMode(polyscope::PointRenderMode::Quad);
    polyscope::show();
    return 0;
}
