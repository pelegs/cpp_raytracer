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
    vec3_d p0;
    vec4_d normal_form;
    vec3_d n;

  public:
    Plane(){

        normal_form = glm::normalize(vec4_d(0.0, 1.0, 0.0, 1.0));
        n = normal_form;
    }

    Plane(const mat33_d &pnts){
        // init from three points
        p0 = pnts[0];
        vec3_d v1 = glm::normalize(pnts[1] - pnts[0]);
        vec3_d v2 = glm::normalize(pnts[2] - pnts[0]);
        vec3_d v3 = glm::cross(v1, v2);
        double d = -1.0*glm::dot(v3, pnts[0]);
        this->normal_form = glm::normalize(glm::vec4(v3, d));
        this->n = this->normal_form;
    }

    Plane(const vec4_d &normal_form){
        // init from normal form only
        this->normal_form = glm::normalize(normal_form);
        this->n = normal_form;
        auto t = first_non_zero_element(n);
        int i = std::get<0>(t);
        double ai = std::get<1>(t);
        double d = this->normal_form[3];
        this->p0 = {0.0, 0.0, 0.0};
        this->p0[i] = -ai/d;
    }

    Plane(const vec3_d &normal, const vec3_d &p0){
        double d = -1.0*glm::dot(normal, p0);
        this->normal_form = glm::normalize(glm::vec4(normal, d));
    }

    vec4_d get_normal_form() const {
        return this->normal_form;
    }

    vec3_d get_normal() const {
        return this->n;
    }

    vec3_d get_p0() const {
        return this->p0;
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
    double d = normal_form[3];
    double a = glm::dot(line_start, plane_normal) + d;
    return a/d;
}


// ---------- //
// -- MAIN -- //
// ---------- //

int main(){
    vec4_d normal_form = {1.0, 3.0, -7.0, 21.0};
    // vec3_d p0 = {}
    Plane plane(normal_form);
    std::cout << glm::to_string(plane.get_normal()) << ", " << glm::to_string(plane.get_p0()) << std::endl;
    return 0;
}
