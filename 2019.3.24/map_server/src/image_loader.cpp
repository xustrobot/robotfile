/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
/ *

*威洛车库有限公司版权所有(c) 2008

*版权所有。

*

*以源代码和二进制形式重新分发和使用，有或没有

*如符合下列条件，可作修改:

*

* *重新发布的源代码必须保留上述版权

*注意，此条件列表及以下免责声明。

* *以二进制形式重新发行必须复制上述版权

*注意，此条件清单及以下免责声明载于

*分发文件和/或提供的其他材料。

* *没有柳树车库公司的名字，也没有它的名字

*投稿人可用于支持或推广源自

*本软件未经特别事先书面许可。

*

*本软件由版权所有人和贡献者“按原样”提供

*以及任何明示或暗示的保证，包括但不限于

*隐含的适销性和适合特定用途的保证

*是否认的。在任何情况下，版权所有人或贡献者都不能

*对任何直接、间接、附带、特殊、示范或

*间接损害赔偿(包括但不限于采购

*替代商品或服务;丧失使用、数据或利润;或业务

(中断)然而造成的和基于任何理论的责任，无论是否

*合同、严格责任或侵权(包括过失或其他)

*因使用本软件而产生的任何后果，即使事先已被告知

这类损害的可能性。
 */

/*此文件包含用于将图像加载为映射的辅助函数。
 * This file contains helper functions for loading images as maps.
 *
 * Author: Brian Gerkey
 */

#include <cstring>
#include <stdexcept>

#include <stdlib.h>
#include <stdio.h>

// We use SDL_image to load the image from disk 我们使用sdl_image从磁盘加载图像
#include <SDL/SDL_image.h>
 
// Use Bullet's Quaternion object to create one from Euler angles 使用项目符号的四元数对象从欧拉角度创建一个。 

#include <LinearMath/btQuaternion.h>

#include "map_server/image_loader.h"

// compute linear index for given map coords 计算给定地图坐标的线性索引
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

namespace map_server
{

void
loadMapFromFile(nav_msgs::GetMap::Response* resp,
                const char* fname, double res, bool negate,
                double occ_th, double free_th, double* origin,
                MapMode mode)
{
  SDL_Surface* img;

  unsigned char* pixels;
  unsigned char* p;
  unsigned char value;
  int rowstride, n_channels, avg_channels;
  unsigned int i,j;
  int k;
  double occ;
  int alpha;
  int color_sum;
  double color_avg;

  // Load the image using SDL.  If we get NULL back, the image load failed. 使用SDL加载图像。如果返回空值，则图像加载失败。
  if(!(img = IMG_Load(fname)))
  {
    std::string errmsg = std::string("failed to open image file \"") +
            std::string(fname) + std::string("\": ") + IMG_GetError();
    throw std::runtime_error(errmsg);
  }

  // Copy the image data into the map structure   将图像数据复制到地图结构中 
  resp->map.info.width = img->w;
  resp->map.info.height = img->h;
  resp->map.info.resolution = res;
  resp->map.info.origin.position.x = *(origin);
  resp->map.info.origin.position.y = *(origin+1);
  resp->map.info.origin.position.z = 0.0;
  btQuaternion q;
  // setEulerZYX(yaw, pitch, roll) 设置Euler ZYX偏航、俯仰、侧倾
  q.setEulerZYX(*(origin+2), 0, 0);
  resp->map.info.origin.orientation.x = q.x();
  resp->map.info.origin.orientation.y = q.y();
  resp->map.info.origin.orientation.z = q.z();
  resp->map.info.origin.orientation.w = q.w();

  // Allocate space to hold the data 分配存储数据的空间
  resp->map.data.resize(resp->map.info.width * resp->map.info.height);

  // Get values that we'll need to iterate through the pixels 获取需要迭代像素的值
  rowstride = img->pitch;
  n_channels = img->format->BytesPerPixel;

  // NOTE: Trinary mode still overrides here to preserve existing behavior. 注意：三元模式仍然在这里覆盖以保留现有的行为。
  // Alpha will be averaged in with color channels when using trinary mode. 使用三元模式时，alpha将在颜色通道中平均。
  if (mode==TRINARY || !img->format->Amask)
    avg_channels = n_channels;
  else
    avg_channels = n_channels - 1;

  // Copy pixel data into the map structure 将像素数据复制到地图结构中
  pixels = (unsigned char*)(img->pixels); 
  for(j = 0; j < resp->map.info.height; j++)
  {
    for (i = 0; i < resp->map.info.width; i++)
    {
      // Compute mean of RGB for this pixel 计算该像素的RGB平均值
      p = pixels + j*rowstride + i*n_channels;
      color_sum = 0;
      for(k=0;k<avg_channels;k++)
        color_sum += *(p + (k));
      color_avg = color_sum / (double)avg_channels;

      if (n_channels == 1)
          alpha = 1;
      else
          alpha = *(p+n_channels-1);

      if(negate)
        color_avg = 255 - color_avg;

      if(mode==RAW){
          value = color_avg;
          resp->map.data[MAP_IDX(resp->map.info.width,i,resp->map.info.height - j - 1)] = value;
          continue;
      }


      // If negate is true, we consider blacker pixels free, and whiter
      // pixels occupied.  Otherwise, it's vice versa. 如果负数为真，则认为较黑的像素是空闲的，而较白的像素是占用的。反之亦然。
      occ = (255 - color_avg) / 255.0;

      // Apply thresholds to RGB means to determine occupancy values for
      // map.  Note that we invert the graphics-ordering of the pixels to
      // produce a map with cell (0,0) in the lower-left corner.
     //对RGB方法应用阈值以确定地图的占用率值。请注意，我们颠倒像素的图形顺序，以生成一个左下角具有单元格（0,0）的地图。
      if(occ > occ_th)
        value = +100;
      else if(occ < free_th)
        value = 0;
      else if(mode==TRINARY || alpha < 1.0)
        value = -1;
      else {
        double ratio = (occ - free_th) / (occ_th - free_th);
        value = 99 * ratio;
      }

      resp->map.data[MAP_IDX(resp->map.info.width,i,resp->map.info.height - j - 1)] = value;
    }
  }

  SDL_FreeSurface(img);
}

}
