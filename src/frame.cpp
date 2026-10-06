/* 
This file is part of KILVO, the implementation of our paper.

Derived from FAST-LIVO2.
Modified for KILVO by Jixin Gao <gaojixin99@163.com>, 2026,
for more information, see <https://github.com/JixinGao/KILVO>.

If you use this code, please cite the relevant publications as
listed on the above website.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include <boost/bind.hpp>
#include "feature.h"
#include "frame.h"
#include "visual_point.h"
#include <stdexcept>
#include <vikit/math_utils.h>
#include <vikit/performance_monitor.h>
#include <vikit/vision.h>

int Frame::frame_counter_ = 0;

Frame::Frame(vk::AbstractCamera *cam, const cv::Mat &img)
    : id_(frame_counter_++), 
      cam_(cam)
{
  initFrame(img);
}

Frame::~Frame()
{
  std::for_each(fts_.begin(), fts_.end(), [&](Feature *i) { delete i; });
}

void Frame::initFrame(const cv::Mat &img)
{
  if (img.empty()) { throw std::runtime_error("Frame: provided image is empty"); }

  if (img.cols != cam_->width() || img.rows != cam_->height())
  {
    throw std::runtime_error("Frame: provided image has not the same size as the camera model");
  }

  if (img.type() != CV_8UC1) { throw std::runtime_error("Frame: provided image is not grayscale"); }

  img_ = img;
}

/// Utility functions for the Frame class
namespace frame_utils
{

void createImgPyramid(const cv::Mat &img_level_0, int n_levels, ImgPyr &pyr)
{
  pyr.resize(n_levels);
  pyr[0] = img_level_0;
  for (int i = 1; i < n_levels; ++i)
  {
    pyr[i] = cv::Mat(pyr[i - 1].rows / 2, pyr[i - 1].cols / 2, CV_8U);
    vk::halfSample(pyr[i - 1], pyr[i]);
  }
}

} // namespace frame_utils
