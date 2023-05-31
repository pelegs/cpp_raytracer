#include <iostream>
#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/ext.hpp>
#include <vector>
#include <tuple>
#include <limits>


typedef glm::vec<3, double> vec3_d;
typedef glm::vec<4, double> vec4_d;
typedef glm::mat<3, 3, double> mat33_d;


double inf = std::numeric_limits<double>::infinity();


std::tuple<int, double> first_non_zero_element(const vec3_d &v){
    for (int i=0; i<3; i++)
        if ( v[i] != 0.0 ) return std::make_tuple(i, v[i]);
    return std::make_tuple(-1, 0.0);
}

class Line{
    vec3_d start;
    vec3_d dir;

  public:
    Line(){
        start = vec3_d{0.0, 0.0, 0.0};
        dir = vec3_d{0.0, 0.0, 0.0};
    }

    Line(const vec3_d &init_pos, const vec3_d &direction){
        start = init_pos;
        dir = glm::normalize(direction);
    }

    vec3_d point_at(double t){
        return this->start + t*this->dir;
    }

    vec3_d get_start() const {
        return this->start;
    }

    vec3_d get_direction() const {
        return this->dir;
    }
};

class Plane{
    mat33_d points;
    vec4_d normal_form;
    vec3_d normal;

  public:
    Plane(){
        points = mat33_d(1.0);
        normal_form = glm::normalize(vec4_d(0.0, 1.0, 0.0, 1.0));
        normal = normal_form;
    }

    Plane(const mat33_d &ps){
        // Initializes 
        points = ps;
        vec3_d v1 = glm::normalize(points[1] - points[0]);
        vec3_d v2 = glm::normalize(points[2] - points[0]);
        vec3_d n = glm::cross(v1, v2);
        double d = -1.0*glm::dot(n, ps[0]);
        normal_form = glm::normalize(glm::vec4(n, d));
        normal = normal_form;
    }

    Plane(const glm::vec4 &normform){
        normal_form = glm::normalize(normform);
        normal = normal_form;
        // double d = normal_form[3];
        // std::tuple<int, double> fnze = first_non_zero_element(normal_form);
        // int i;
        // double xi;
        // std::tie(i, xi) = fnze;
        // vec3_d p = {0.0, 0.0, 0.0};
        // vec3_d n = normal_form;
        // if ( i >= 0 && i <= 2) {
        //     p[i] = -d/xi;
        //     vec3_d v2 = glm::cross(v1, n);
        // }
        //
    }

    Plane(const vec3_d &n, const vec3_d &p){
        double d = -1.0*glm::dot(n, p);
        normal_form = glm::normalize(glm::vec4(n, d));
    }

    glm::vec4 get_normal_form() const {
        return this->normal_form;
    }

    glm::vec3 get_normal() const {
        return this->normal;
    }
};


// ----------- //
// -- FUNCS -- //
// ----------- //

double line_plane_intersection(const Line &line, const Plane &plane){
    vec3_d line_dir = line.get_direction();
    vec4_d normal_form = plane.get_normal_form();
    vec3_d plane_normal = normal_form;
    double n_dot_l = glm::dot(line_dir, plane_normal);

    // Plane and line are parallel
    if ( n_dot_l == 0.0 )
        return inf;

    // Plane and line intersect
    vec3_d line_start = line.get_start();
    double d = plane_normal[3];
    double a = glm::dot(line_start, plane_normal) + d;
    return -a/d;
}


// ---------- //
// -- MAIN -- //
// ---------- //

int main(){
    vec4_d nf = {0.0, 1.0, 0.0, 1.0};
    Plane plane(nf);
    std::cout << glm::to_string(plane.get_normal()) << std::endl;
    return 0;
}
