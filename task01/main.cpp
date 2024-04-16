#include <cstdio>
#include <iostream>
#include <cassert>
#include <vector>
#include <filesystem>
//
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif 

/**
 * @brief compute the area of a triangle
 * @return area
 */
float area_of_a_triangle(
    float x0, float y0,
    float x1, float y1,
    float x2, float y2) {
  return ((x1 - x2) * (y0 - y2) - (x0 - x2) * (y1 - y2)) * 0.5f;
}

/**
 * @brief draw a triangle using the barycentric coordinates
 */
void draw_triangle(
    float x0, float y0,
    float x1, float y1,
    float x2, float y2,
    std::vector<unsigned char> &img_data, unsigned int width, unsigned int height,
    unsigned char brightness) {
  for (unsigned int ih = 0; ih < height; ++ih) {
    for (unsigned int iw = 0; iw < width; ++iw) {
      const auto x = (float) iw + 0.5f;
      const auto y = (float) ih + 0.5f;
      const auto a01 = area_of_a_triangle(x, y, x0, y0, x1, y1);
      const auto a12 = area_of_a_triangle(x, y, x1, y1, x2, y2);
      const auto a20 = area_of_a_triangle(x, y, x2, y2, x0, y0);
      if (a01 > 0.f && a12 > 0.f && a20 > 0.f) {
        img_data[ih * height + iw] = brightness;
      }
    }
  }
}

/**
 * @brief draw a triangle using the barycentric coordinates
 * @param polygon_xy xy coordinates of the corners of the polygon (counter clockwise order)
 * @param brightness brightness of the painted pixel
 */
void draw_polygon(
    const std::vector<float> &polygon_xy,
    std::vector<unsigned char> &img_data, unsigned int width, unsigned int height,
    unsigned int brightness) {
  for (unsigned int ih = 0; ih < height; ++ih) {
    for (unsigned int iw = 0; iw < width; ++iw) {
      const auto x = float(iw) + 0.5f; // x-coordinate of the center of the pixel
      const auto y = float(ih) + 0.5f; // y-coordinate of the center of the pixel
      const unsigned int num_vtx = polygon_xy.size() / 2;
      float winding_number = 0.0;
      for (unsigned int iedge = 0; iedge < num_vtx; ++iedge) {
        unsigned int i0_vtx = iedge;
        unsigned int i1_vtx = (iedge + 1) % num_vtx;
        // positions of the end points of the edge relative to (x,y)
        float p0x = polygon_xy[i0_vtx * 2 + 0] - x;
        float p0y = polygon_xy[i0_vtx * 2 + 1] - y;
        float p1x = polygon_xy[i1_vtx * 2 + 0] - x;
        float p1y = polygon_xy[i1_vtx * 2 + 1] - y;
        // write a few lines of code to compute winding number (hint: use atan2)

        // calculating variables
        float scalar_product_p0_p1 = (p0x*p1x+p0y*p1y);
        float norm_p0 = sqrt(p0x*p0x + p0y*p0y);
        float norm_p1 = sqrt(p1x*p1x + p1y*p1y);
        float cross_product_p0_p1_scalar_product =p0y*p1x-p0x*p1y;
        float cos_theta = scalar_product_p0_p1/(norm_p0*norm_p1);
        float sin_theta = cross_product_p0_p1_scalar_product/(norm_p0*norm_p1);
        // Winding number for each edge
        float theta = atan2(sin_theta,cos_theta);
        winding_number += theta/(2*M_PI);

      }
      const int int_winding_number = int(std::round(winding_number));
      if (int_winding_number == 1 ) { // if (x,y) is inside the polygon
        img_data[ih*width + iw] = brightness;
      }
    }
  }
}

/**
 * @brief draw a line using DDA algorithm
 * @param x0 x-coordinate of the first end point
 * @param y0 y-coordinate of the first end point
 * @param x1 x-coordinate of the second end point
 * @param y1 y-coordinate of the second end point
 * @param brightness brightness of the painted pixel
 */
void dda_line(
    float x0, float y0,
    float x1, float y1,
    std::vector<unsigned char> &img_data,
    unsigned int width,
    unsigned char brightness ) {
  auto dx = x1 - x0;
  auto dy = y1 - y0;
  // write some code below to paint pixel on the line with color `brightness`
  // slope
  float m = dy/dx;
  if ( abs(m) < 1 ) {
      for (unsigned int i = 0; i < floor(abs(x1-x0)); ++i) {
      if(x1-x0<0){
        auto p_ix = x0-i;
        auto p_iy =y0-m*i;
        int int_p_ix = floor(p_ix);
        int int_p_iy = floor(p_iy);
        img_data[int_p_iy*width + int_p_ix] = brightness;

      }
      else{
        auto p_ix = x0+i;
        auto p_iy =y0+m*i;
        int int_p_ix = floor(p_ix);
        int int_p_iy = floor(p_iy);
        img_data[int_p_iy*width + int_p_ix] = brightness;
      }
      
  }
  }
  else{
    m = dx/dy;
    for (unsigned int i = 0; i < floor(abs(y1-y0)); ++i) {
      if(y1-y0<0){
        auto p_ix = x0-m*i;
        auto p_iy =y0-i;
        int int_p_ix = floor(p_ix);
        int int_p_iy = floor(p_iy);
        img_data[int_p_iy*width + int_p_ix] = brightness;

      }
      else{
        auto p_ix = x0+m*i;
        auto p_iy =y0+i;
        int int_p_ix = floor(p_ix);
        int int_p_iy = floor(p_iy);
        img_data[int_p_iy*width + int_p_ix] = brightness;
      }
      

  }
  }
  
  

}

int main() {
  constexpr unsigned int width = 100;
  constexpr unsigned int height = 100;
  std::vector<unsigned char> img_data(width * height, 255); // white initial image
  draw_triangle(5., 5., 15., 45., 45., 15., img_data, width, height, 128);
  draw_polygon(
      {55., 5.,75., 75.,15., 55.,15., 95.,95., 95.,95., 5.}, // the pix coordinates of the corners of the polygon
      img_data, width, height, // image data, size
      200); // brightness to draw
  dda_line(50.0, 50.0, 72.8, 10.0, img_data, width, 0); // right-down
  dda_line(50.0, 50.0, 90.0, 27.2, img_data, width, 0); // right-down
  dda_line(50.0, 50.0, 72.8, 90.0, img_data, width, 0); // right-up
  dda_line(50.0, 50.0, 90.0, 72.8, img_data, width, 0); // right-up
  dda_line(50.0, 50.0, 10.0, 72.8, img_data, width, 0); // left-up
  dda_line(50.0, 50.0, 27.2, 90.0, img_data, width, 0); // left-up
  dda_line(50.0, 50.0, 10.0, 27.2, img_data, width, 0); // left-down
  dda_line(50.0, 50.0, 27.2, 10.0, img_data, width, 0); // left-down
  dda_line(50.0, 50.0, 90.0, 50.0, img_data, width, 0); // right
  dda_line(50.0, 50.0, 50.0, 90.0, img_data, width, 0); // up
  dda_line(50.0, 50.0, 10.0, 50.0, img_data, width, 0); // left
  dda_line(50.0, 50.0, 50.0, 10.0, img_data, width, 0); // down
  // save gray scale image with 1 byte depth
  stbi_write_png(
      (std::filesystem::path(PROJECT_SOURCE_DIR) / "output.png").string().c_str(),
      width, height, 1,img_data.data(), width);
}
