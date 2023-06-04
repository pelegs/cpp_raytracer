#include <iostream>
#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/ext.hpp>
#include <glm/gtc/epsilon.hpp>
#include <vector>
#include <tuple>
#include <limits>


typedef glm::vec<3, double> vec3_d;
typedef glm::vec<4, double> vec4_d;
typedef glm::mat<3, 3, double> mat33_d;


const double PERCISION = 1.0E-7;


double inf = std::numeric_limits<double>::infinity();


int first_non_zero_index(const vec3_d &v){
    for ( int i=0; i<3; i++ )
        if ( v[i] != 0.0 ) return i;
    return -1; // should raise a value error
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
        this->normal_form = vec4_d(v3, d);
        this->n = vec3_d(this->normal_form);
        // TODO: check that this is correct!
    }

    Plane(const vec4_d &normal_form){
        // init from normal form only
        vec3_d norm = normal_form;
        double nflen = glm::length(norm);
        this->n = glm::normalize(norm);
        double d = normal_form[3]/nflen;
        // std::cout << glm::to_string(this->n) << ": " << nflen << " -> d=" << d << std::endl;
        this->normal_form = vec4_d(this->n, d);
        int i = first_non_zero_index(normal_form);
        this->p0 = {0.0, 0.0, 0.0};
        this->p0[i] = -d/normal_form[i];
    }

    Plane(const vec3_d &normal, const vec3_d &p0){
        double nlen = glm::length(normal);
        this->normal = glm::normalize(normal);
        // TODO!!!
    }

    vec4_d get_normal_form() const {
        return this->normal_form;
    }

    vec3_d get_n() const {
        return this->n;
    }

    vec3_d get_p0() const {
        return this->p0;
    }

    bool point_in_plane(const vec3_d &p) const{
        return glm::epsilonEqual(glm::dot(p, this->n)+this->normal_form[3], 0.0, PERCISION);
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
    vec3_d p = {-2,3,4.};
    Plane plane(normal_form);
    double f = 1.0/plane.get_n()[0];
    std::cout << glm::to_string(f*plane.get_normal_form()) << std::endl;
    std::cout << plane.point_in_plane(p) << std::endl;

    return 0;
}
