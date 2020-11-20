#include "points_gen.h"
#include <vector>

using namespace Points_gen;

std::vector<Point> points_List::gen_Point_list(Point* p){
    std::vector<Point> vec;
    vec.push_back(*p);

    return vec;
}
    