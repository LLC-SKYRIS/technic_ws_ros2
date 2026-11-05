#pragma once

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

void _drawPlanarBoard(cv::aruco::Board *_board, cv::Size outSize, cv::OutputArray _img,
                      int marginSize, int borderBits, bool drawAxis); // editorconfig-checker-disable-line
void _drawAxis(cv::InputOutputArray image, cv::InputArray cameraMatrix, cv::InputArray distCoeffs,
               cv::InputArray rvec, cv::InputArray tvec, float length); // editorconfig-checker-disable-line
