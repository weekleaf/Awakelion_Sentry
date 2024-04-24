/*
 * @Description: This is a ros_control learning project!
 * @Author: Liu Biao
 * @Date: 2022-09-05 03:24:50
 * @LastEditTime: 2023-06-02 21:56:20
 * @FilePath: /TUP-Vision-2023-Based/src/global_user/include/global_user/global_user.hpp
 */
#ifndef GLOBAL_USER_HPP_
#define GLOBAL_USER_HPP_

#include <string>
#include <vector>
#include <thread>
#include <memory>
#include <iterator>
#include <unistd.h>
#include <future>
#include <fstream>
#include <yaml-cpp/yaml.h>

//opencv
#include <opencv2/opencv.hpp>

//eigen
#include <Eigen/Dense>
#include <Eigen/Core>

//linux
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>

// daheng
#define DAHENG_IMAGE_WIDTH 1280 
#define DAHENG_IMAGE_HEIGHT 1024
// hik
#define HIK_IMAGE_WIDTH 1440     
#define HIK_IMAGE_HEIGHT 1080
// usb
#define USB_IMAGE_WIDTH 640     
#define USB_IMAGE_HEIGHT 480
// mvs
#define MVS_IMAGE_WIDTH 1280     
#define MVS_IMAGE_HEIGHT 1024

using namespace std;
using namespace Eigen;
using namespace cv;
namespace global_user
{   
    /**
     * @brief Global variables and funcs.
     * 
     */


    struct ImageSize
    {
        int width;
        int height;

        ImageSize()
        {
            this->width = DAHENG_IMAGE_WIDTH;
            this->height = DAHENG_IMAGE_HEIGHT;
        }
    };

    struct GridAndStride
    {
        int grid0;
        int grid1;
        int stride;
    };

    struct ObjectBase
    {
        int id;
        int color;
        double conf;
        std::string key;
        Eigen::Vector3d armor3d_cam;
        Eigen::Vector3d armor3d_world;
        Eigen::Vector3d euler;
        Eigen::Matrix3d rmat;
    };
    
    struct Object
    {
        cv::Rect_<float> rect;
        int cls;
        int color;
        float prob;
        std::vector<cv::Point2f> pts;
    };

    template<typename T>
    bool initMatrix(Eigen::MatrixXd &matrix,std::vector<T> &vector)
    {
        int cnt = 0;
        for(int row = 0;row < matrix.rows();row++)
        {
            for(int col = 0;col < matrix.cols();col++)
            {
                matrix(row,col) = vector[cnt];
                cnt++;
            }
        }
        return true;
    }

    float calcTriangleArea(cv::Point2d pts[3]);
    float calcTetragonArea(cv::Point2d pts[4]);
    double rangedAngleRad(double &angle);

    std::string symbolicToReal(std::string path);
    std::string relativeToFull(std::string relative, std::string src);
    std::string treeToPath(std::vector<std::string> &tree);
    std::string getParent(std::string path);

    std::vector<std::string> readLines(std::string file_path);
    std::vector<std::string> generatePathTree(std::string path);

    Eigen::Vector3d rotationMatrixToEulerAngles(Eigen::Matrix3d &R);
    Eigen::Vector3d calcDeltaEuler(Eigen::Vector3d euler1, Eigen::Vector3d euler2);
    Eigen::AngleAxisd eulerToAngleAxisd(Eigen::Vector3d euler);
    Eigen::Matrix3d eulerToRotationMatrix(Eigen::Vector3d &theta);
    float calcDistance(cv::Point2d& p1, cv::Point2d& p2);

    bool autoLabel(bool& is_init, cv::Mat &img, ofstream &file, string &path_name, int64_t &timestamp, int &id, int &color, vector<cv::Point2d> &apex2d, cv::Point2i &roi_offset, cv::Size2i &input_size);

    bool isPnpSolverValidation(Eigen::Vector3d& point3d);
    bool isAngleSolverValidataion(Eigen::Vector2d& angle2d);
    void drawAimCrossCurve(cv::Mat& src);

    // bool checkDivergence(const MatrixXd& residual, const MatrixXd& S, double threshold);
    // bool checkDivergence(double residual, double threshold, vector<double>& variances, int window_size);
    // bool checkDivergence(const MatrixXd& F, const MatrixXd& P, const MatrixXd& H, const MatrixXd& R);
    bool checkDivergence(const MatrixXd& statePre, const MatrixXd& stateCovPre, const MatrixXd& H, const MatrixXd& R, const VectorXd& measurement);
} // namespace global_user

#endif
