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
typedef glm::mat<2, 2, double> mat22_d;
typedef glm::mat<3, 3, double> mat33_d;
typedef glm::mat<4, 4, double> mat44_d;


// General constants
const double PERCISION = 1.0E-7;
const double inf = std::numeric_limits<double>::infinity();
const double pi = glm::pi<double>();
const double two_pi = 2.0*pi;
const double half_pi = glm::half_pi<double>();
const double third_pi = pi/3.0;
const double quarter_pi = half_pi/2.0;
const double sixth_pi = third_pi/2.0;
const double sqrt_2 = glm::root_two<double>();
const double one_over_sqrt_2 = 1.0/sqrt_2;

// Vector and matrix constants
const vec2_d Zero2 = {.0, .0};
const vec3_d Zero3 = {.0, .0, .0};
const vec4_d Zero4 = {.0, .0, .0, .0};
const vec3_d X_ = {1.0, .0, .0};
const vec3_d Y_ = {.0, 1.0, .0};
const vec3_d Z_ = {.0, .0, 1.0};
const vec2_d O2_ = {.0, .0};
const vec3_d O3_ = {.0, .0, .0};
const vec4_d O4_ = {.0, .0, .0, .0};
const mat22_d I2 = glm::mat2(1.0);
const mat33_d I3 = glm::mat3(1.0);
const mat44_d I4 = glm::mat4(1.0);


// ------------------- //
// -- GENERAL FUNCS -- //
// ------------------- //

int first_non_zero_index(const vec3_d &v)
{
    for ( int i=0; i<3; i++ )
        if ( v[i] != 0.0 ) return i;
    throw std::logic_error("zero vector can't be used for direction");
}

vec3_d reflect_ray(const vec3_d &r, const vec3_d &n)
{
    // Returns the direction of a reflected ray r by a point with normal n.
    // Note: assumes ray direction r is normalized!
    return r - 2.0 * n * glm::dot(r, n);
}

double angle_between(const vec3_d &u, const vec3_d &v)
{
    double sqrt_norms = glm::sqrt(glm::length2(u)*glm::length2(v)); // saves a single sqrt calculation
    return glm::acos(glm::dot(u, v)/sqrt_norms);
}

vec3_d axis_between(const vec3_d &u, const vec3_d &v)
{
    return glm::normalize(glm::cross(u, v));
}

vec3_d rand_vec_solid_angle(const double &th)
{
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

vec3_d rand_vec_solid_angle_in_direction(const vec3_d &dir, const double &th)
{
    double phi = angle_between(Z_, dir);
    vec3_d ax = axis_between(Z_, dir);
    vec3_d random = rand_vec_solid_angle(th);
    return glm::rotate(random, phi, ax);
}


class Line
{
    vec3_d start;
    vec3_d dir;

  public:
    Line()
    {
        start = vec3_d{0.0, 0.0, 0.0};
        dir = vec3_d{0.0, 0.0, 0.0};
    }

    Line(const vec3_d &init_pos, const vec3_d &direction)
    {
        start = init_pos;
        dir = glm::normalize(direction);
    }

    vec3_d point_at(double t)
    {
        return this->start + t*this->dir;
    }

    vec3_d get_start() const
    {
        return this->start;
    }

    vec3_d get_direction() const
    {
        return this->dir;
    }
};


class Plane
{
    vec3_d n, v1, v2;
    vec4_d normal_form;
    mat33_d pts;

  public:
    Plane()
    {
        normal_form = vec4_d(0.0, 1.0, 0.0, -1.0);
        n = normal_form;
        vec3_d p0 = {1.0, 1.0, 0.0};
        vec3_d p1 = {0.0, 1.0, 1.0};
        vec3_d p2 = {0.0, 1.0, 0.0};
        pts = {p0, p1, p2};
    }

    // TODO: correct Plane init!!

    Plane(const mat33_d &pts)
    {
        // init from three points
        this->pts = pts;
        v1 = pts[1] - pts[0];
        v2 = pts[2] - pts[0];
        n = glm::normalize(glm::cross(v1, v2));
        double d = -1.0*glm::dot(n, pts[0]);
        normal_form = vec4_d(n, d);
        // TODO: check that this is correct!
    }

    // Plane(const vec4_d &normal_form)
    // {
    //     // init from normal form only
    //     vec3_d norm = normal_form;
    //     double nflen = glm::length(norm);
    //     this->n = glm::normalize(norm);
    //     double d = normal_form[3]/nflen;
    //     this->normal_form = vec4_d(this->n, d);
    //     int i = first_non_zero_index(normal_form);
    //     this->p0 = {0.0, 0.0, 0.0};
    //     this->p0[i] = -d/this->n[i];
    // }
    //
    // Plane(const vec3_d &normal, const vec3_d &p0)
    // {
    //     this->n = glm::normalize(normal);
    //     double d = -glm::dot(p0, this->n);
    //     this->normal_form = vec4_d(this->n, d);
    //     this->p0 = p0;
    // }

    vec4_d get_normal_form() const
    {
        return this->normal_form;
    }

    vec3_d get_n() const
    {
        return this->n;
    }

    mat33_d get_pts() const
    {
        return this->pts;
    }

    bool point_in_plane(const vec3_d &p) const
    {
        return glm::epsilonEqual(glm::dot(p, this->n)+this->normal_form[3], 0.0, PERCISION);
    }
};


class Hittable {
    int id;
    int type;

  public:
    vec3_d reflect(const vec3_d &pos, const vec3_d &dir)
    {
        return -Y_;
    }
};


class Sphere: public Hittable
{
    vec3_d center;
    double radius;
    double radius2;

  public:
    Sphere()
    {
        center = O3_;
        radius = 1.0;
        radius2 = 1.0;
    }

    Sphere(const vec3_d &p, const double &r)
    {
        this->center = p;
        this->radius = r;
        this->radius2 = r*r;
    }

    bool point_on_surface(const vec3_d &p) const
    {
        double dist2 = glm::distance2(this->center, p);
        return glm::epsilonEqual(dist2, this->radius2, PERCISION);
    }

    vec3_d normal_at_surface(const vec3_d &p) const
    {
        // Should check if p on surface?..
        return glm::normalize(p-this->center);
    }

    vec3_d reflect(const vec3_d &pos, const vec3_d &dir) const
    {
        return reflect_ray(dir, this->normal_at_surface(pos));
    }
};


// class Triangle: public Plane, public Hittable
// {
//     mat33_d pts;
//
//   public:
//     Triangle(): Plane()
//     {
//         std::cout << glm::to_string(this->get_normal_form()) << std::endl;
//     }
//
//     Triangle(const mat33_d &pts): Plane(pts)
//     {
//         this->pts = {this->get_p0(), pts[1], pts[2]}; // yes, this is strange.
//     }
//
//     mat33_d get_pts() const
//     {
//         return this->pts;
//     }
// };


// ------------------------ //
// -- FUNCS WITH CLASSES -- //
// ------------------------ //

double line_plane_intersection(const Line &line, const Plane &plane)
{
    double l_dot_n = glm::dot(line.get_direction(), plane.get_n());
    if ( glm::epsilonEqual(l_dot_n, 0.0, PERCISION) )
        return inf;
    double a = glm::dot(line.get_start()-plane.get_pts()[0], plane.get_n());
    return -a/l_dot_n;
}

// ---------- //
// -- MAIN -- //
// ---------- //

int main()
{
    // srand(time(NULL));
    vec3_d p0 = {1.0, 0.0, 5.0};
    vec3_d p1 = {0.0, 1.0, 5.0};
    vec3_d p2 = {1.0, 1.0, 5.0};
    mat33_d pts = {p0, p1, p2};
    // Triangle t(pts);
    Plane t;
    std::cout << glm::to_string(t.get_n()) << std::endl;

    return 0;
}
