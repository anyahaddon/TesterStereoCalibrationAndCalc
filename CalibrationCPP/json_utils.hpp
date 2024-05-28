#ifndef JSON_UTILS_H
#define JSON_UTILS_H

#include <opencv2/opencv.hpp>
#include "json.hpp"

using namespace cv;
using json = nlohmann::json;

void to_json(json& j, const Mat& mat);
void from_json(const json& j, Mat& mat);

#endif // JSON_UTILS_H
