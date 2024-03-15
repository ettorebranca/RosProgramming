#pragma once
#include <boost/filesystem.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <queue>


namespace fs = boost::filesystem;
using namespace Eigen;
using namespace std;


struct GridMap {
    int rows,cols;
    vector<uint8_t> gridMapArray;
    vector<vector<uint8_t>> gridMapMatrix;
    cv::Mat gridMapImage;
    Matrix<float, Dynamic, Dynamic> distanceMap;
    double minDistance, maxDistance;
    pair<int, int> start, goal;


    GridMap();

    void loadFromImage(const fs::path& folderName);
    bool isImageFile(const fs::path& filePath);
    void printGridMap();
    void printDistanceMap();
    void displayDistanceMap();
    void cv2vec(const cv::Mat& src, std::vector<uint8_t>& dst);
    void vec2cv(const std::vector<uint8_t>& src, cv::Mat& dst);
    void computeDistanceMap();
    void loadFromVec(vector<uint8_t>,int rows,int cols);

    int actionCost(int x,int y);
    bool isValid(int x,int y);
    int heuristic(int x,int y);
    vector<pair<int, int>> findPath(pair<int, int> Start, pair<int, int> Goal);
    void displayPath(vector<pair<int, int>> path);

};

