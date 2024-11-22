#include "file_system.hpp"
#include <Eigen/Eigen>
#include <System.h>
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <ctime>
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <regex.h>
#include <regex>
#include <signal.h>
#include <sstream>
#include <stdlib.h>

bool b_continue_session;

void exit_loop_handler(int s) {
  cout << "Finishing session" << endl;
  b_continue_session = false;
}

// Function to extract number from filename
uint64_t extractTimestamp(const std::string &filename) {
  std::regex pattern(R"((mono_right|mono_left|depth|rgb)_(\d+)\.png)");
  std::smatch match;

  if (std::regex_match(filename, match, pattern)) {
    if (match.size() == 3) {
      return std::stoull(match[2].str());
    }
  }
  return std::numeric_limits<uint64_t>::max();
}

void loadLeftImages(const string &dataRootFolder,
                    std::vector<string> &vstrImages,
                    std::vector<double> &vTimeStamps) {
  vTimeStamps.reserve(5000);
  vstrImages.reserve(5000);
  auto imgFolder = stlplus::folder_down(
      stlplus::folder_down(dataRootFolder, "mono"), "left");
  auto allFiles = stlplus::folder_all(imgFolder);

  for (auto filename : allFiles) {
    if (filename.find("mono_left_") != std::string::npos) {
      double timestamp = double(extractTimestamp(filename)) / 1e3; // ms -> s
      filename = stlplus::create_filespec(imgFolder, filename);
      vTimeStamps.push_back(timestamp);
      vstrImages.push_back(filename);
    }
  }
}

void loadRightImages(const string &dataRootFolder,
                     std::vector<string> &vstrImages,
                     std::vector<double> &vTimeStamps) {
  vTimeStamps.reserve(5000);
  vstrImages.reserve(5000);
  auto imgFolder = stlplus::folder_down(
      stlplus::folder_down(dataRootFolder, "mono"), "right");
  auto allFiles = stlplus::folder_all(imgFolder);

  for (auto filename : allFiles) {
    if (filename.find("mono_right_") != std::string::npos) {
      double timestamp = double(extractTimestamp(filename)) / 1e3; // ms -> s
      filename = stlplus::create_filespec(imgFolder, filename);
      vTimeStamps.push_back(timestamp);
      vstrImages.push_back(filename);
    }
  }
}

void loadFrontLeftImages(const string &dataRootFolder,
                     std::vector<string> &vstrImages,
                     std::vector<double> &vTimeStamps) {
  vTimeStamps.reserve(5000);
  vstrImages.reserve(5000);
  auto imgFolder = stlplus::folder_down(
      stlplus::folder_down(dataRootFolder, "mono"), "front_left");
  auto allFiles = stlplus::folder_all(imgFolder);

  for (auto filename : allFiles) {
    if (filename.find("mono_left_") != std::string::npos) {
      double timestamp = double(extractTimestamp(filename)) / 1e3; // ms -> s
      filename = stlplus::create_filespec(imgFolder, filename);
      vTimeStamps.push_back(timestamp);
      vstrImages.push_back(filename);
    }
  }
}

void loadFrontRightImages(const string &dataRootFolder,
                     std::vector<string> &vstrImages,
                     std::vector<double> &vTimeStamps) {
  vTimeStamps.reserve(5000);
  vstrImages.reserve(5000);
  auto imgFolder = stlplus::folder_down(
      stlplus::folder_down(dataRootFolder, "mono"), "front_right");
  auto allFiles = stlplus::folder_all(imgFolder);

  for (auto filename : allFiles) {
    if (filename.find("mono_right_") != std::string::npos) {
      double timestamp = double(extractTimestamp(filename)) / 1e3; // ms -> s
      filename = stlplus::create_filespec(imgFolder, filename);
      vTimeStamps.push_back(timestamp);
      vstrImages.push_back(filename);
    }
  }
}

void readInImuFromFile(std::string fileName, std::vector<double> &vTimeStamps,
                       std::vector<cv::Point3f> &vAcc,
                       std::vector<cv::Point3f> &vGyro) {
  std::ifstream file(fileName);
  if (!file.is_open()) {
    std::cerr << "cannot open file: " << fileName << std::endl;
    return;
  }

  std::string line;
  double timestamp;
  cv::Point3f rawAcc;
  cv::Point3f rawGyro;
  cv::Point3f calibratedAcc;
  cv::Point3f calibratedGyro;
  Eigen::Vector3d linearAcc;
  Eigen::Vector3d angularVel;
  Eigen::Matrix3d accT;
  Eigen::Matrix3d gyroT;
  Eigen::Vector3d accBias;
  Eigen::Vector3d gyroBias;
  // clang-format off
    accT << 1.00108, 0.000914835, 0.00955, 
            0, 0.9969, -0.00216, 
            0, 0, 1.0005;
    gyroT << 1.00412, -0.01214, 0.00409, 
            -0.00787, 0.99352, -0.0088, 
            -0.01437,-0.0145, 0.9892;
    accBias << -0.000739, -0.0205, -0.00239;
    gyroBias << -0.00654, 0.0088, -0.0030;
  // clang-format on
  while (std::getline(file, line)) {
    if (line.find("TimeStamp:") != std::string::npos) {
      timestamp = double(std::stoll(line.substr(line.find(":") + 1))) / 1e3;
    } else if (line.find("gx:") != std::string::npos) {
      rawGyro.x = std::stof(line.substr(line.find(":") + 1));
    } else if (line.find("gy:") != std::string::npos) {
      rawGyro.y = std::stof(line.substr(line.find(":") + 1));
    } else if (line.find("gz:") != std::string::npos) {
      rawGyro.z = std::stof(line.substr(line.find(":") + 1));
    } else if (line.find("ax:") != std::string::npos) {
      rawAcc.x = std::stof(line.substr(line.find(":") + 1));
    } else if (line.find("ay:") != std::string::npos) {
      rawAcc.y = std::stof(line.substr(line.find(":") + 1));
    } else if (line.find("az:") != std::string::npos) {
      rawAcc.z = std::stof(line.substr(line.find(":") + 1));
    } else if (line.find("mx:") != std::string::npos) {
      linearAcc << rawAcc.x, -rawAcc.z, -rawAcc.y;
      angularVel << rawGyro.x, -rawGyro.z, -rawGyro.y;

      linearAcc = accT * linearAcc - accBias;
      angularVel = gyroT * angularVel - gyroBias;

      calibratedAcc.x = linearAcc.x() * 9.8;
      calibratedAcc.y = linearAcc.y() * 9.8;
      calibratedAcc.z = linearAcc.z() * 9.8;

      calibratedGyro.x = angularVel.x();
      calibratedGyro.y = angularVel.y();
      calibratedGyro.z = angularVel.z();

      vAcc.push_back(calibratedAcc);
      vGyro.push_back(calibratedGyro);
      vTimeStamps.push_back(timestamp);
    }
  }
  file.close();
}

void sortPath(std::vector<std::string> &vStrPath,
              std::vector<double> &vTimestamp) {
  std::vector<int> indices(vTimestamp.size());
  for (size_t i = 0; i < indices.size(); ++i) {
    indices[i] = i;
  }
  std::sort(indices.begin(), indices.end(), [&vTimestamp](int i, int j) {
    return vTimestamp[i] < vTimestamp[j];
  });
  std::vector<string> sortedStrPath(vStrPath.size());
  for (size_t i = 0; i < indices.size(); ++i) {
    sortedStrPath[i] = vStrPath[indices[i]];
  }
  std::sort(vTimestamp.begin(), vTimestamp.end());
  vStrPath = sortedStrPath;
}

int main(int argc, char **argv) {
  if (argc != 4) {
    cerr << endl << "Usage: ./revopoint_pop3.yaml path_to_vocabulary " << endl;
    return 1;
  }

  struct sigaction sigIntHandler;

  sigIntHandler.sa_handler = exit_loop_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;

  sigaction(SIGINT, &sigIntHandler, NULL);
  b_continue_session = true;
  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::CAMERA_RIG,true,0);
  // ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true, 0);
  std::string dataRootDir = argv[3];
  std::vector<std::string> vStrLeftImages;
  std::vector<std::string> vStrRightImages;
  std::vector<std::string> vStrFrontLeftImages;
  std::vector<std::string> vStrFrontRightImages;
  std::vector<double> vLeftTimestamps;
  std::vector<double> vRightTimestamps;
  std::vector<double> vFrontLeftTimestamps;
  std::vector<double> vFrontRightTimestamps;
  std::vector<double> vImuTimeStamps;
  std::vector<cv::Point3f> vAcc;
  std::vector<cv::Point3f> vGyro;
  std::string imuPath = stlplus::create_filespec(dataRootDir, "imu.txt");
  loadLeftImages(dataRootDir, vStrLeftImages, vLeftTimestamps);
  loadRightImages(dataRootDir, vStrRightImages, vRightTimestamps);
  loadFrontLeftImages(dataRootDir, vStrFrontLeftImages, vFrontLeftTimestamps);
  loadFrontRightImages(dataRootDir, vStrFrontRightImages,
                       vFrontRightTimestamps);
  sortPath(vStrLeftImages, vLeftTimestamps);
  sortPath(vStrRightImages, vRightTimestamps);
  sortPath(vStrFrontLeftImages, vFrontLeftTimestamps);
  sortPath(vStrFrontRightImages, vFrontRightTimestamps);

  cv::Mat leftImg;
  cv::Mat rightImg;
  cv::Mat frontLeftImg;
  cv::Mat frontRightImg;
  vector<ORB_SLAM3::IMU::Point> vImuMeas;
  while (!SLAM.isShutDown()) {
    for (int id = 10; id < vStrLeftImages.size(); id++) {
      // Read image from file
      leftImg = cv::imread(vStrLeftImages[id], cv::IMREAD_UNCHANGED);
      rightImg = cv::imread(vStrRightImages[id], cv::IMREAD_UNCHANGED);
      frontLeftImg = cv::imread(vStrFrontLeftImages[id], cv::IMREAD_UNCHANGED);
      frontRightImg = cv::imread(vStrFrontRightImages[id], cv::IMREAD_UNCHANGED);
      double timestamp = vLeftTimestamps[id];

      if (leftImg.empty()) {
        cerr << endl
             << "Failed to load image at: " << vStrLeftImages[id] << endl;
        return 1;
      }
      if (rightImg.empty()) {
        cerr << endl
             << "Failed to load image at: " << vStrRightImages[id] << endl;
        return 1;
      }
      if (frontLeftImg.empty()) {
        cerr << endl
             << "Failed to load image at: " << vStrFrontLeftImages[id] << endl;
        return 1;
      }
      if (frontRightImg.empty()) {
        cerr << endl
             << "Failed to load image at: " << vStrFrontRightImages[id] << endl;
        return 1;
      }

      // Create SLAM system. It initializes all system threads and gets ready to
      // process frames.
      float imageScale = SLAM.GetImageScale();
      SLAM.TrackCameraRig(frontLeftImg, frontRightImg, leftImg, rightImg,
                      timestamp);
    }
  }
  cout << "System shutdown!\n";
}