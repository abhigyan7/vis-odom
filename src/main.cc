#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"

int main(int argc, char** argv )
{
    if ( argc != 2 )
    {
        printf("usage: DisplayImage.out <Image_Path>\n");
        return -1;
    }
    polyscope::init();
    std::vector<glm::vec3> points;

    // generate points
    for (size_t i = 0; i < 3000; i++) {
      points.push_back(
          glm::vec3{polyscope::randomUnit() - .5,
                    polyscope::randomUnit() - .5,
                    polyscope::randomUnit() - .5});
    }

    // visualize!
    polyscope::PointCloud* psCloud = polyscope::registerPointCloud("really great points", points);

    // set some options
    psCloud->setPointRadius(0.02);
    psCloud->setPointRenderMode(polyscope::PointRenderMode::Quad);
    polyscope::show();
    cv::Mat image;
    image = cv::imread( argv[1], 1 );

    if ( !image.data )
    {
        printf("No image data \n");
        return -1;
    }

    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
    cv::imshow("Display Image", image);
    cv::waitKey(0);
    return 0;
}
