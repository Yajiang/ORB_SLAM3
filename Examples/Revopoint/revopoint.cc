// /**
//  * This file is part of ORB-SLAM3
//  *
//  * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez
//  * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
//  * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
//  * University of Zaragoza.
//  *
//  * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
//  * the terms of the GNU General Public License as published by the Free Software
//  * Foundation, either version 3 of the License, or (at your option) any later
//  * version.
//  *
//  * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
//  * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
//  * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//  *
//  * You should have received a copy of the GNU General Public License along with
//  * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
//  */

// #include "file_system.hpp"
// #include <Eigen/Eigen>
// #include <System.h>
// #include <algorithm>
// #include <chrono>
// #include <condition_variable>
// #include <ctime>
// #include <fstream>
// #include <iostream>
// #include <opencv2/core/core.hpp>
// #include <regex.h>
// #include <regex>
// #include <signal.h>
// #include <sstream>
// #include <stdlib.h>

// bool b_continue_session;

// void exit_loop_handler(int s) {
//   cout << "Finishing session" << endl;
//   b_continue_session = false;
// }

// // Function to extract number from filename
// uint64_t ExtractTimestamp(const std::string &filename) {
//   std::regex pattern(R"((mono_left|depth|rgb)_(\d+)\.png)");
//   std::smatch match;

//   if (std::regex_match(filename, match, pattern)) {
//     if (match.size() == 3) {
//       return std::stoull(match[2].str());
//     }
//   }
//   return std::numeric_limits<uint64_t>::max();
// }

// void LoadImages(const string &dataRootFolder, std::vector<string> &vstrImages,
//                 std::vector<double> &vTimeStamps) {
//   vTimeStamps.reserve(5000);
//   vstrImages.reserve(5000);
//   auto rgbDepthFolder = stlplus::folder_down(dataRootFolder, "rgb_depth");
//   auto allFiles = stlplus::folder_all(rgbDepthFolder);

//   for (auto filename : allFiles) {
//     if (filename.find("mono_left_") != std::string::npos) {
//       double timestamp = double(ExtractTimestamp(filename)) / 1e3; // ms -> s
//       filename = stlplus::create_filespec(rgbDepthFolder, filename);
//       vTimeStamps.push_back(timestamp);
//       vstrImages.push_back(filename);
//     }
//   }
// }

// void LoadDepths(const string &dataRootFolder, std::vector<string> &vstrImages,
//                 std::vector<double> &vTimeStamps) {
//   vTimeStamps.reserve(5000);
//   vstrImages.reserve(5000);
//   auto rgbDepthFolder = stlplus::folder_down(dataRootFolder, "rgb_depth");
//   auto allFiles = stlplus::folder_all(rgbDepthFolder);

//   for (auto filename : allFiles) {
//     if (filename.find("depth_") != std::string::npos) {
//       double timestamp = double(ExtractTimestamp(filename)) / 1e3; // ms -> s
//       filename = stlplus::create_filespec(rgbDepthFolder, filename);
//       vTimeStamps.push_back(timestamp);
//       vstrImages.push_back(filename);
//     }
//   }
// }

// void readInImuFromFile(std::string fileName, std::vector<double> &vTimeStamps,
//                        std::vector<cv::Point3f> &vAcc,
//                        std::vector<cv::Point3f> &vGyro) {
//   std::ifstream file(fileName);
//   if (!file.is_open()) {
//     std::cerr << "cannot open file: " << fileName << std::endl;
//     return;
//   }

//   std::string line;
//   double timestamp;
//   cv::Point3f rawAcc;
//   cv::Point3f rawGyro;
//   cv::Point3f calibratedAcc;
//   cv::Point3f calibratedGyro;
//   Eigen::Vector3d linearAcc;
//   Eigen::Vector3d angularVel;
//   Eigen::Matrix3d accT;
//   Eigen::Matrix3d gyroT;
//   Eigen::Vector3d accBias;
//   Eigen::Vector3d gyroBias;
//   // clang-format off
//     accT << 1.00108, 0.000914835, 0.00955, 
//             0, 0.9969, -0.00216, 
//             0, 0, 1.0005;
//     gyroT << 1.00412, -0.01214, 0.00409, 
//             -0.00787, 0.99352, -0.0088, 
//             -0.01437,-0.0145, 0.9892;
//     accBias << -0.000739, -0.0205, -0.00239;
//     gyroBias << -0.00654, 0.0088, -0.0030;
//   // clang-format on
//   while (std::getline(file, line)) {
//     if (line.find("TimeStamp:") != std::string::npos) {
//       timestamp = double(std::stoll(line.substr(line.find(":") + 1))) / 1e3;
//     } else if (line.find("gx:") != std::string::npos) {
//       rawGyro.x = std::stof(line.substr(line.find(":") + 1));
//     } else if (line.find("gy:") != std::string::npos) {
//       rawGyro.y = std::stof(line.substr(line.find(":") + 1));
//     } else if (line.find("gz:") != std::string::npos) {
//       rawGyro.z = std::stof(line.substr(line.find(":") + 1));
//     } else if (line.find("ax:") != std::string::npos) {
//       rawAcc.x = std::stof(line.substr(line.find(":") + 1));
//     } else if (line.find("ay:") != std::string::npos) {
//       rawAcc.y = std::stof(line.substr(line.find(":") + 1));
//     } else if (line.find("az:") != std::string::npos) {
//       rawAcc.z = std::stof(line.substr(line.find(":") + 1));
//     } else if (line.find("mx:") != std::string::npos) {
//       linearAcc << rawAcc.x, -rawAcc.z, -rawAcc.y;
//       angularVel << rawGyro.x, -rawGyro.z, -rawGyro.y;

//       linearAcc = accT * linearAcc - accBias;
//       angularVel = gyroT * angularVel - gyroBias;

//       calibratedAcc.x = linearAcc.x() * 9.8;
//       calibratedAcc.y = linearAcc.y() * 9.8;
//       calibratedAcc.z = linearAcc.z() * 9.8;

//       calibratedGyro.x = angularVel.x();
//       calibratedGyro.y = angularVel.y();
//       calibratedGyro.z = angularVel.z();

//       vAcc.push_back(calibratedAcc);
//       vGyro.push_back(calibratedGyro);
//       vTimeStamps.push_back(timestamp);
//     }
//   }
//   file.close();
// }

// void sortPath(std::vector<std::string> &vStrPath,
//               std::vector<double> &vTimestamp) {
//   std::vector<int> indices(vTimestamp.size());
//   for (size_t i = 0; i < indices.size(); ++i) {
//     indices[i] = i;
//   }
//   std::sort(indices.begin(), indices.end(),
//             [&vTimestamp](int i, int j) { return vTimestamp[i] < vTimestamp[j]; });
//   std::vector<string> sortedStrPath(vStrPath.size());
//   for (size_t i = 0; i < indices.size(); ++i) {
//     sortedStrPath[i] = vStrPath[indices[i]];
//   }
//   std::sort(vTimestamp.begin(),vTimestamp.end());
//   vStrPath = sortedStrPath;
// }

// int main(int argc, char **argv) {
//   if (argc != 3) {
//     cerr << endl << "Usage: ./revopoint_pop3.yaml path_to_vocabulary " << endl;
//     return 1;
//   }

//   struct sigaction sigIntHandler;

//   sigIntHandler.sa_handler = exit_loop_handler;
//   sigemptyset(&sigIntHandler.sa_mask);
//   sigIntHandler.sa_flags = 0;

//   sigaction(SIGINT, &sigIntHandler, NULL);
//   b_continue_session = true;
//   ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_RGBD, true,
//                          0);

//   double offset = 0; // ms

//   std::string dataRootDir =
//       "/home/yajianghuang/Revopoint/data/orb_slam3_data/data_1";
//   std::vector<std::string> vStrImages;
//   std::vector<double> vImgTimestamps;
//   std::vector<std::string> vStrDepths;
//   std::vector<double> vDepthTimestamps;
//   std::vector<double> vImuTimeStamps;
//   std::vector<cv::Point3f> vAcc;
//   std::vector<cv::Point3f> vGyro;
//   std::string imuPath = stlplus::create_filespec(dataRootDir, "imu.txt");
//   LoadImages(dataRootDir, vStrImages, vImgTimestamps);
//   LoadDepths(dataRootDir, vStrDepths, vDepthTimestamps);
//   readInImuFromFile(imuPath, vImuTimeStamps, vAcc, vGyro);
//   sortPath(vStrImages, vImgTimestamps);
//   sortPath(vStrDepths, vDepthTimestamps);

//   int firstImu = 0;
//   int firstDepth = 0;
//   while (vImuTimeStamps[firstImu] <= vImgTimestamps[0])
//     firstImu++;
//   firstImu--; // first imu measurement to be considered
//   firstImu +=100;

//   while (vDepthTimestamps[firstDepth] <= vImgTimestamps[0])
//     firstDepth++;
//   firstDepth--; // first imu measurement to be considered

//   cv::Mat img;
//   cv::Mat depth;
//   vector<ORB_SLAM3::IMU::Point> vImuMeas;
//   while (!SLAM.isShutDown()) {
//     for (int id = 10; id < vStrImages.size(); id++) {
//       // Read image from file
//       img = cv::imread(vStrImages[id], cv::IMREAD_UNCHANGED);
//       depth = cv::imread(vStrDepths[id], cv::IMREAD_UNCHANGED);
//       cv::resize(img,img,cv::Size(320,200));

//       double timestamp = vImgTimestamps[id];

//       if (img.empty()) {
//         cerr << endl << "Failed to load image at: " << vStrImages[id] << endl;
//         return 1;
//       }
//       if (depth.empty()) {
//         cerr << endl << "Failed to load image at: " << vStrDepths[id] << endl;
//         return 1;
//       }

//       // Load imu measurements from previous frame
//       vImuMeas.clear();

//       if (id > 0) {
//         while (vImuTimeStamps[firstImu] <= vImgTimestamps[id]) {
//           vImuMeas.push_back(ORB_SLAM3::IMU::Point(
//               vAcc[firstImu].x, vAcc[firstImu].y, vAcc[firstImu].z,
//               vGyro[firstImu].x, vGyro[firstImu].y, vGyro[firstImu].z,
//               vImuTimeStamps[firstImu]));
//           firstImu++;
//         }
//       }

//       // Create SLAM system. It initializes all system threads and gets ready to
//       // process frames.
//       float imageScale = SLAM.GetImageScale();
//       // Clear IMU vectors
//       SLAM.TrackRGBD(img, depth, timestamp, vImuMeas);
//     }
//   }
//   cout << "System shutdown!\n";
// }