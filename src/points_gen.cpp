#include "points_gen.h"
#include <vector>
#include <iostream>
#include <cmath>

using namespace Points_gen;

std::vector<Point> points_List::gen_Point_list(){
    std::vector<Point> vec;

    //std::cout << vec.capacity() << std::endl;
    double m = 5.0;
    double n = 5.0;
    double robot_radius = 0.175;
    int count = 0;
    
    for (double i = robot_radius; i < m; i+=2*robot_radius)
    {
        //std::cout << count % 2 << std::endl;
        //std::cout << fmod(i,robot_radius) << std::endl;
        if (count%2==0){
            
            for(double j = robot_radius; j < n; j += 2*robot_radius){
                Point p = {i,j};
                std::cout << p.x << "," << p.y << std::endl;
                //std::cin.get();
                vec.push_back(p);
            } 
        }else{
            for(double j = n-robot_radius; j > 0; j -= 2*robot_radius){
                Point p = {i,j};
                std::cout << p.x << "," << p.y << std::endl;
                //std::cin.get();
                vec.push_back(p);
            } 
        }
        count++;
    }
    
    std::cin.get();
    /*
    for(double i = robot_radius; i < m-robot_radius; i + 2*robot_radius){
        std::cout << i << std::endl;
        for(double j = robot_radius; j < n - robot_radius; j + 2*robot_radius){
            Point p = {i,j};
            std::cout << p.x << "," << p.y << std::endl;
            std::cin.get();
            vec.push_back(p);
        }
    }
    */


    return vec;
}
    