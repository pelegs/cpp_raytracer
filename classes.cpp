#include <iostream>
#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/ext.hpp>
#include <glm/gtc/epsilon.hpp>
#include <vector>
#include <tuple>
#include <limits>
#include <stdexcept>


typedef glm::vec<2, double> vec2_d;
typedef glm::vec<3, double> vec3_d;
typedef glm::vec<4, double> vec4_d;
typedef glm::mat<3, 3, double> mat33_d;


// General constants
const double PERCISION = 1.0E-7;
const double inf = std::numeric_limits<double>::infinity();
const double pi = glm::pi<double>();
const double two_pi = 2.0*pi;
const double half_pi = glm::half_pi<double>();
const double third_pi = pi/3.0;
const double quarter_pi = half_pi/2.0;
const double sixth_pi = third_pi/2.0;

// Vector and matrix constants
const vec2_d Zero2 = {.0, .0};
const vec3_d Zero3 = {.0, .0, .0};
const vec4_d Zero4 = {.0, .0, .0, .0};
const vec3_d X_ = {1.0, .0, .0};
const vec3_d Y_ = {.0, 1.0, .0};
const vec3_d Z_ = {.0, .0, 1.0};
const mat33_d I3 = {X_, Y_, Z_};


int first_non_zero_index(const vec3_d &v) {
    for ( int i=0; i<3; i++ )
        if ( v[i] != 0.0 ) return i;
    throw std::logic_error("zero vector can't be used for direction");
}


class Line{
    vec3_d start;
    vec3_d dir;

  public:
    Line() {
        start = vec3_d{0.0, 0.0, 0.0};
        dir = vec3_d{0.0, 0.0, 0.0};
    }

    Line(const vec3_d &init_pos, const vec3_d &direction) {
        start = init_pos;
        dir = glm::normalize(direction);
    }

    vec3_d point_at(double t) {
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
    Plane() {

        normal_form = glm::normalize(vec4_d(0.0, 1.0, 0.0, 1.0));
        n = normal_form;
    }

    Plane(const mat33_d &pnts) {
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

    Plane(const vec4_d &normal_form) {
        // init from normal form only
        vec3_d norm = normal_form;
        double nflen = glm::length(norm);
        this->n = glm::normalize(norm);
        double d = normal_form[3]/nflen;
        this->normal_form = vec4_d(this->n, d);
        int i = first_non_zero_index(normal_form);
        this->p0 = {0.0, 0.0, 0.0};
        this->p0[i] = -d/this->n[i];
    }

    Plane(const vec3_d &normal, const vec3_d &p0) {
        double nlen = glm::length(normal);
        this->n = glm::normalize(normal);
        double d = -glm::dot(p0, this->n);
        this->normal_form = vec4_d(this->n, d);
        this->p0 = p0;
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

    bool point_in_plane(const vec3_d &p) const {
        return glm::epsilonEqual(glm::dot(p, this->n)+this->normal_form[3], 0.0, PERCISION);
    }
};


// ----------- //
// -- FUNCS -- //
// ----------- //

double line_plane_intersection(const Line &line, const Plane &plane) {
    double l_dot_n = glm::dot(line.get_direction(), plane.get_n());
    if ( glm::epsilonEqual(l_dot_n, 0.0, PERCISION) )
        return inf;
    double a = glm::dot(line.get_start()-plane.get_p0(), plane.get_n());
    return -a/l_dot_n;
}

vec3_d reflect(const vec3_d &r, const vec3_d &n) {
    // Returns the direction of a reflected ray r by a point with normal n.
    // Note: assumes ray direction r is normalized!
    return r - 2.0 * n * glm::dot(r, n);
}

double angle_between(const vec3_d &u, const vec3_d &v) {
    double sqrt_norms = glm::sqrt(glm::length2(u)*glm::length2(v)); // saves a single sqrt calculation
    return glm::acos(glm::dot(u, v)/sqrt_norms);
}

vec3_d axis_between(const vec3_d &u, const vec3_d &v) {
    return glm::normalize(glm::cross(u, v));
}

vec3_d rand_vec_solid_angle(const double &th) {
    // Generate a random point on a sphere (uniformly distributed)
    vec3_d u = glm::sphericalRand(1.0);

    // Find angle and axis from pt to Z_=(0,0,1)
    double phi = angle_between(u, Z_);
    vec3_d ax = axis_between(u, Z_);

    // If point already inside solid angle th, return it
    if (phi < th) return u;

    // Otherwise, rotate pt around ax in a random angle psi ∈ [phi-th, phi],
    // bringing it inside solid angle th
    double psi = glm::linearRand(phi-th, phi);
    vec3_d r = glm::rotate(u, psi, ax);

    return r;
}

vec3_d rand_vec_solid_angle_in_direction(const vec3_d &dir, const double &th) {
    double phi = angle_between(Z_, dir);
    vec3_d ax = axis_between(Z_, dir);
    vec3_d random = rand_vec_solid_angle(th);
    return glm::rotate(random, phi, ax);
}


// ---------- //
// -- MAIN -- //
// ---------- //

int main() {
    srand(time(NULL));
    vec3_d rand;
    vec3_d dir = glm::sphericalRand(1.0);
    int N = 1000;
    for (int i=0; i<N; i++) {
        rand = rand_vec_solid_angle_in_direction(dir, quarter_pi);
        std::cout << printf("%0.5f %0.5f %0.5f", rand.x, rand.y, rand.z) << std::endl;
    }

    return 0;
}
