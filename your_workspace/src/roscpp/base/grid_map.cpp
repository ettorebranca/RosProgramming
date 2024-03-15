#include "grid_map.h"

using namespace std;
namespace fs = boost::filesystem;
using namespace Eigen;


GridMap::GridMap(){
    cerr << "GridMap initialization" << endl;
    };

bool GridMap::isImageFile(const fs::path& filePath) {
    std::string extension = filePath.extension().string();
    
    return (extension == ".jpg" || extension == ".jpeg" || extension == ".png" || extension == ".bmp" || extension == ".gif");
};

void GridMap::loadFromVec(std::vector<uint8_t> src, int Rows,int Cols){
    gridMapArray = src;
    rows = Rows;
    cols = Cols;
    vec2cv(gridMapArray,gridMapImage);
}

void GridMap::loadFromImage(const fs::path& folderName){
    fs::path imagePath;
    try {
        int imageCount = 0;
        fs::directory_iterator endIter;
        
        for (fs::directory_iterator dirIter(folderName); dirIter != endIter; ++dirIter) {
            if (fs::is_regular_file(dirIter->path()) && isImageFile(dirIter->path())) {
                ++imageCount;
                imagePath = dirIter->path();
            }
        }

        if (imageCount == 1) {
            std::cout << "Found exactly one image file in the folder." << std::endl;
        } else if (imageCount > 1) {
            std::cout << "Found more than one image file in the folder." << std::endl;
        } else {
            std::cout << "No image file found in the folder." << std::endl;
        }
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
        throw;
    }

    cerr << "loading [" << imagePath.string() << "]" << endl;
    cv::Mat m = cv::imread(imagePath.string());
    if (m.rows == 0) {
    throw std::runtime_error("unable to load image");
    }
    cv::Mat loaded_image;
    cv::cvtColor(m, loaded_image, cv::COLOR_BGR2GRAY);
    cerr << loaded_image.at<uchar>(0,0);

    rows = loaded_image.rows;
    cols = loaded_image.cols;
    
    cv2vec(loaded_image, gridMapArray);
}

void GridMap::printGridMap() {
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            std::cout << static_cast<int>(gridMapArray[i*cols +j]) << " ";
        }
        std::cout << std::endl;
    }
}

void GridMap::printDistanceMap() {
    
    for (int i = 0; i < distanceMap.rows(); ++i) {
        for (int j = 0; j < distanceMap.cols(); ++j) {
            std::cout << static_cast<int>(distanceMap(i, j)) << " ";
        }
        std::cout << std::endl;
    }
}

void GridMap::displayDistanceMap() {
    cv::Mat distanceImage(distanceMap.rows(), distanceMap.cols(), CV_8UC1);
    for (int i = 0; i < distanceMap.rows(); ++i) {
        for (int j = 0; j < distanceMap.cols(); ++j) {
            
            distanceImage.at<uchar>(i, j) = static_cast<uchar>(distanceMap(i, j) * 255.0 / maxDistance);
        }
    }

    cv::namedWindow("Distance Map", cv::WINDOW_AUTOSIZE);
    cv::imshow("Distance Map", distanceImage);
    cv::waitKey(0);
    cv::destroyWindow("Distance Map");
}

void GridMap::cv2vec(const cv::Mat& src, std::vector<uint8_t>& dst) {
        dst.resize(cols*rows);
        for (int i = 0; i < src.rows; ++i) {
            for (int j = 0; j < src.cols; ++j) {
                if (src.at<uchar>(i, j) > 230) {
                    dst[i * src.cols + j] = 255;
                } else {
                    dst[i * src.cols + j] = 0;
                }
            }
        }
    }

void GridMap::vec2cv(const std::vector<uint8_t>& src, cv::Mat& dst){
    dst.create(rows, cols, CV_8UC1);

    for (int i = 0; i < dst.rows; ++i) {
        for (int j = 0; j < dst.cols; ++j) {
            dst.at<uchar>(i, j) = src[i * dst.cols + j];
        }
    }
}

void GridMap::computeDistanceMap() {
    cv::Mat gridmapCV(rows, cols, CV_8U);
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            gridmapCV.at<uchar>(i, j) = gridMapArray[i * cols + j];
        }
    }

    cv::Mat distanceTransform;
    cv::distanceTransform(gridmapCV, distanceTransform, cv::DIST_L2, cv::DIST_MASK_PRECISE);

    cv::minMaxLoc(distanceTransform, &minDistance, &maxDistance);

    
    distanceMap.resize(distanceTransform.rows, distanceTransform.cols);
    for (int i = 0; i < distanceTransform.rows; ++i) {
        for (int j = 0; j < distanceTransform.cols; ++j) {
            distanceMap(i, j) = distanceTransform.at<float>(i, j);
        }
    }
}

int GridMap::actionCost(int x,int y){
    int distanceCost = exp(0.5*(maxDistance - distanceMap(x, y))); 
}
bool GridMap::isValid(int x,int y) {
    return x >= 0 && x < rows && y >= 0 && y < cols && gridMapArray[x*cols+y] == 255;
}
int GridMap::heuristic(int x,int y){
    return abs(x - goal.first) + abs(y - goal.second);
}

vector<pair<int, int>> GridMap::findPath(pair<int, int> Start, pair<int, int> Goal) {

    start = Start;
    goal = Goal;

    vector<pair<int, int>> path;

    if(gridMapArray[start.first*cols+start.second]!=255){
        cerr<<"start position is not traversale  value:";
        cerr<<gridMapArray[start.first*cols+start.second];
        return path;
    }
    if(gridMapArray[goal.first*cols+goal.second]!=255){
        cerr<<"goal position "<<goal.first<<"  "<<goal.second<<" is not traversale  value:";
        cerr<<to_string(gridMapArray[goal.first*cols+goal.second]);
        return path;
    }

    vector<vector<bool>> visited(rows, vector<bool>(cols,false));
    vector<vector<pair<int, int>>> parent(rows, vector<pair<int, int>>(cols, {-1, -1}));

    
    vector<vector<int>> gScore(rows, vector<int>(cols, INT_MAX));
    
    vector<vector<int>> fScore(rows, vector<int>(cols, INT_MAX));
    
    gScore[start.first][start.second] = 0;
    
    fScore[start.first][start.second] = heuristic(start.first,start.second);

    std::set<pair<int, int>> openSet;
    openSet.insert(start);
    vector<pair<int, int>> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

    int min_fScore,tentative_gScore;
    pair<int, int> min_fScoreNode;
    pair<int, int> current,neighbour;
    
    int current_fScore;

    while (!openSet.empty()) {
        min_fScore = INT_MAX;
        for (const auto& node : openSet) {
            current_fScore = fScore[node.first][node.second]; 
            
            if (current_fScore < min_fScore) {
                min_fScore = current_fScore;
                min_fScoreNode = node; 
            }
        }
        
        current = min_fScoreNode;

        if (current == goal){
            cerr<<"path found"<<endl;
            while (current != start) {
                path.push_back(current);
                current = parent[current.first][current.second];
            }
            path.push_back(start);
            reverse(path.begin(), path.end());
            return path;
        }

        openSet.erase(current);
        visited[current.first][current.second]=true;
        
        for (auto direction : directions ){
            neighbour = {current.first+direction.first,current.second+direction.second};
            if (isValid(neighbour.first,neighbour.second)){
                tentative_gScore = gScore[current.first][current.second]+actionCost(neighbour.first,neighbour.second);
                
                if (tentative_gScore < gScore[neighbour.first][neighbour.second]){
                    parent[neighbour.first][neighbour.second] = current;
                    gScore[neighbour.first][neighbour.second] = tentative_gScore;
                    fScore[neighbour.first][neighbour.second] = gScore[neighbour.first][neighbour.second] + heuristic(neighbour.first,neighbour.second);
                    
                    openSet.insert(neighbour);

                }

            }

        }
    }
    cerr<<"path not found";
    return path;

}

void GridMap::displayPath(vector<pair<int, int>> path){
    cv::Mat gridMapImageWithPath(rows, cols, CV_8UC3);
    
    cv::cvtColor(gridMapImage, gridMapImageWithPath, cv::COLOR_GRAY2RGB);

    for (auto cell:path){
        cv::Vec3b& pixel = gridMapImageWithPath.at<cv::Vec3b>(cell.first, cell.second);

        pixel[0] = 0;  // Blue
        pixel[1] = 0;    // Green
        pixel[2] = 255;    // Red
    }
    
    cv::namedWindow("Path", cv::WINDOW_AUTOSIZE);
    cv::imshow("Path", gridMapImageWithPath);
    cv::waitKey(0); 
    cv::destroyWindow("Path");
}


