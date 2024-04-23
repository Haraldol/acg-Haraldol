#include <filesystem>
// #include <experimental/filesystem> // uncomment here if the <filesystem> cannot be included above
//
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include "Eigen/Core"
//
#include "parse_svg.h"

/***
 * signed area of a triangle connecting points (p0, p1, p2) in counter-clockwise order.
 * @param p0 1st point xy-coordinate
 * @param p1 2nd point xy-coordinate
 * @param p2 3rd point xy-coordinate
 * @return signed area (float)
 */
float area(
    const Eigen::Vector2f &p0,
    const Eigen::Vector2f &p1,
    const Eigen::Vector2f &p2) {
  const auto v01 = p1 - p0;
  const auto v02 = p2 - p0;
  // return 0.5f * (v01[0] * v02[1] - v01[1] * v02[0]); // right handed coordinate
  return 0.5f * (v01[1] * v02[0] - v01[0] * v02[1]); // left-handed coordinate (because pixel y-coordinate is going down)
}


/***
 * compute number of intersection of a ray against a line segment
 * @param org ray origin
 * @param dir ray direction (unit normal)
 * @param ps one of the two end points
 * @param pe the other end point
 * @return number of intersection
 */
int number_of_intersection_ray_against_edge(
    const Eigen::Vector2f &org,
    const Eigen::Vector2f &dir,
    const Eigen::Vector2f &ps,
    const Eigen::Vector2f &pe) {
  auto a = area(org, org + dir, ps);
  auto b = area(org, pe, org + dir);
  auto c = area(org, ps, pe);
  auto d = area(dir+ps, ps, pe);
  if (a * b > 0.f && d * c < 0.f) { return 1; }
  return 0;
  // the following code was a bug
  //auto d = area(org + dir, ps, pe);
  //if (a * b > 0.f && d * c > 0.f && fabs(d) > fabs(c)) { return 1; }
}

/***
 *
 * @param org ray origin
 * @param dir ray direction (unit vector)
 * @param ps one of the two end points
 * @param pc control point
 * @param pe the other end point
 * @return the number of intersections
 */
int number_of_intersection_ray_against_quadratic_bezier(
    const Eigen::Vector2f &org,
    const Eigen::Vector2f &dir,
    const Eigen::Vector2f &ps,
    const Eigen::Vector2f &pc,
    const Eigen::Vector2f &pe) {
  // comment out below to do the assignment
  //return number_of_intersection_ray_against_edge(org, dir, ps, pe);
  // write some code below to find the intersection between ray and the quadratic

  // Harald Olin: To solve this equation I will use the expression P(t)=q+sv, where P(t) is the cubic bezier curve
  // and q+sv is the ray with origin q, direction v and scale s.
  //  If we start by multiplying LHS and RHS with a vector "n" perpendicular to "v" we get the following expression:
  // P(t)*n=q, which will turn out to nothing other than a 2nd degree polynome which we can solve with the pq-formula

    // Setting up the perpendicular vector
    int number_of_intersections = 0;
    auto dir_perp= dir;
    dir_perp[0] = -1*dir[1];
    dir_perp[1] = dir[0]; 
    // Calculating all the scalar multiplications on the LHS  P(t)*n
    auto a = ps[0]*dir_perp[0]+ps[1]*dir_perp[1];
    auto b = pc[0]*dir_perp[0]+pc[1]*dir_perp[1];
    auto c = pe[0]*dir_perp[0]+pe[1]*dir_perp[1];
    auto d = org[0]*dir_perp[0]+org[1]*dir_perp[1];

    // we then get the polynom a*(1-t)^2+2b(1-t)t+ct^2=d
    // which we can simplyfy to t^2(a-2b+c)+t(-2a+2b)+(a-d)=0
    // now we can divde with the coefficent for t^2 and solve with pq-formula.
    auto denominator = (a-2*b+c);
      // edge case handling division by zero
      if( denominator == 0){
        //solve linear equation t = -(a-d)/(-2a+2b)
        if((-2*a+2*b)!=0){
            auto t = -(a-d)/(-2*a+2*b);
            auto p_t = (1-t)*(1-t)*ps+2*t*(1-t)*pc+t*t*pe;
            auto s_tx = (p_t[0]-org[0])/dir[0];
            auto s_ty = (p_t[1]-org[1])/dir[1];
            if(s_tx == s_ty && s_tx>0){
              return 1;
            } 
        }
        else{
          // we have the situation (a-d)=0 which says nothing about t
          return 0;
        }
      }
      // setting up pq-formula
      else{
        auto p = (-2*a+2*b)/denominator;
        auto q = (a-d)/denominator;
        auto under_root = ((p*p)/4)-q;
        if(under_root<0){
          // if underroot <0 we have a undefined case, which implies no intersection
          return 0;
        }
        // calculating both t
        auto t1 = -1*p/2 + sqrt(under_root);
        auto t2 = -1*p/2 - sqrt(under_root);
        // make sure that 0<t1<1
        if(t1>0 && t1<1){
            // testing if the scale factor s we get from t1 is reasonable
            auto p_t1 = (1-t1)*(1-t1)*ps+2*t1*(1-t1)*pc+t1*t1*pe;
            auto s_t1x = (p_t1[0]-org[0])/dir[0];
            auto s_t1y = (p_t1[1]-org[1])/dir[1];
            // checking if both s are the same, special operation for "float" s-s=0 and that s is nonnegative
            float epsilon = 1e-4;
            if( abs(s_t1x-s_t1y) < epsilon && s_t1x>0){
              number_of_intersections +=1;
              
            } 
        }
        if(t2>0 && t2<1){
          // testing if the scale factor s we get from t2 is reasonable
          auto p_t2 = (1-t2)*(1-t2)*ps+2*t2*(1-t2)*pc+t2*t2*pe;
          auto s_t2x = (p_t2[0]-org[0])/dir[0];
          auto s_t2y = (p_t2[1]-org[1])/dir[1];
          // checking if both s are the same, special operation for "float" s-s=0 and that s is nonnegative
          float epsilon = 1e-4;
          if( abs(s_t2x-s_t2y) < epsilon && s_t2x>0){
            number_of_intersections +=1;
          }  
        }
        return number_of_intersections;
      }
      return number_of_intersections;
}

int main() {
  const auto input_file_path = std::filesystem::path(PROJECT_SOURCE_DIR) / ".." / "asset" / "r.svg";
  const auto [width, height, shape] = acg::svg_get_image_size_and_shape(input_file_path);
  if (width == 0) { // something went wrong in loading the function
    std::cout << "file open failure" << std::endl;
    abort();
  }
  const std::vector<std::string> outline_path = acg::svg_outline_path_from_shape(shape);
  const std::vector<std::vector<acg::Edge>> loops = acg::svg_loops_from_outline_path(outline_path);
  //
  std::vector<unsigned char> img_data(width * height, 255); // grayscale image initialized white
  for (unsigned int ih = 0; ih < height; ++ih) {
    for (unsigned int iw = 0; iw < width; ++iw) {
      const auto org = Eigen::Vector2f(iw + 0.5, ih + 0.5); // pixel center
      const auto dir = Eigen::Vector2f(60., 20.); // search direction
      int count_cross = 0;
      for (const auto &loop: loops) { // loop over loop (letter R have internal/external loops)
        for (const auto &edge: loop) { // loop over edge in the loop
          if (edge.is_bezier) { // in case the edge is a quadratic Bézier
            count_cross += number_of_intersection_ray_against_quadratic_bezier(
                org, dir,
                edge.ps, edge.pc, edge.pe);
          } else { // in case the edge is a line segment
            count_cross += number_of_intersection_ray_against_edge(
                org, dir,
                edge.ps, edge.pe);
          }
        }
      }
      if (count_cross % 2 == 1) { // Jordan's curve theory
        img_data[ih * width + iw] = 0; // paint black if it is inside
      }
    }
  }
  const auto output_file_path = std::filesystem::path(PROJECT_SOURCE_DIR) / "output.png";
  stbi_write_png(output_file_path.string().c_str(), width, height, 1, img_data.data(), width);
}
