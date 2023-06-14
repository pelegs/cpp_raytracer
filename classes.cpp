#include <iostream>
#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/ext.hpp>
#include <glm/gtc/epsilon.hpp>
#include <vector>
#include <tuple>
#include <limits>
#include <stdexcept>
#include <cassert>
// Use (void) to silence unused warnings.
#define assertm(exp, msg) assert(((void)msg, exp))


typedef glm::vec<2, double> vec2d;
typedef glm::vec<3, double> vec3d;
typedef glm::vec<4, double> vec4d;
typedef glm::mat<2, 2, double> mat22d;
typedef glm::mat<3, 3, double> mat33d;
typedef glm::mat<4, 4, double> mat44d;
typedef glm::vec<3, float> color3f;


// Colors
const color3f WHITE = {1.0, 1.0, 1.0};
const color3f BLACK = {0.0, 0.0, 0.0};
const color3f GREY = {0.5, 0.5, 0.5};
const color3f RED = {1.0, 0.0, 0.0};
const color3f GREEN = {0.0, 1.0, 0.0};
const color3f BLUE = {0.0, 0.0, 1.0};
const color3f PURPLE = {1.0, 0.0, 1.0};
const color3f ORANGE = {1.0, 0.5, 0.3};
const int R = 0;
const int G = 1;
const int B = 2;


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
const vec2d Zero2 = {.0, .0};
const vec3d Zero3 = {.0, .0, .0};
const vec4d Zero4 = {.0, .0, .0, .0};
const vec3d X_ = {1.0, .0, .0};
const vec3d Y_ = {.0, 1.0, .0};
const vec3d Z_ = {.0, .0, 1.0};
const vec2d O2_ = {.0, .0};
const vec3d O3_ = {.0, .0, .0};
const vec4d O4_ = {.0, .0, .0, .0};
const mat22d I2 = glm::mat2(1.0);
const mat33d I3 = glm::mat3(1.0);
const mat44d I4 = glm::mat4(1.0);


// ------------------- //
// -- GENERAL FUNCS -- //
// ------------------- //

int first_non_zero_index(const vec3d &v)
{
    for ( int i=0; i<3; i++ )
        if ( v[i] != 0.0 ) return i;
    throw std::logic_error("zero vector can't be used for direction");
}

bool colliniar(const vec3d &a, const vec3d &b)
{
    vec3d an = glm::normalize(a);
    vec3d bn = glm::normalize(b);
    return (glm::all(glm::epsilonEqual(an, bn, PERCISION)) || glm::all(glm::epsilonEqual(an, -1.0*bn, PERCISION)));
}

vec3d reflect_ray(const vec3d &r, const vec3d &n)
{
    // Returns the direction of a reflected ray r by a point with normal n.
    // Note: assumes ray direction r is normalized!
    return r - 2.0 * n * glm::dot(r, n);
}

double angle_between(const vec3d &u, const vec3d &v)
{
    double sqrt_norms = glm::sqrt(glm::length2(u)*glm::length2(v)); // saves a single sqrt calculation
    return glm::acos(glm::dot(u, v)/sqrt_norms);
}

vec3d axis_between(const vec3d &u, const vec3d &v)
{
    return glm::normalize(glm::cross(u, v));
}

vec3d rotate_vec_by_pt(const vec3d v, const double th, const vec3d axis, const vec3d point)
{
    vec3d trans_v = glm::rotate(v-point, th, axis);
    return trans_v+point;
}

vec3d rand_vec_solid_angle(const double &th)
{
    // Generate a random point on a sphere (uniformly distributed)
    vec3d u = glm::sphericalRand(1.0);

    // Find angle and axis from pt to Z_=(0,0,1)
    double phi = angle_between(u, Z_);
    vec3d ax = axis_between(u, Z_);

    // If point already inside solid angle th, return it
    if (phi < th) return u;

    // Otherwise, rotate pt around ax in a random angle psi ∈ [phi-th, phi],
    // bringing it inside solid angle th
    double psi = glm::linearRand(phi-th, phi);
    vec3d r = glm::rotate(u, psi, ax);

    return r;
}

vec3d rand_vec_solid_angle_in_direction(const vec3d &dir, const double &th)
{
    double phi = angle_between(Z_, dir);
    vec3d ax = axis_between(Z_, dir);
    vec3d random = rand_vec_solid_angle(th);
    return glm::rotate(random, phi, ax);
}


// --------------- //
// -- OPERATORS -- //
// --------------- //

mat33d operator+(const mat33d &M, const vec3d &v)
{
    mat33d newM = {M};
    for (int i=0; i<3; i++)
        newM[i] += v;
    return newM;
}

mat33d operator-(const mat33d &M, const vec3d &v)
{
    mat33d newM = {M};
    for (int i=0; i<3; i++)
        newM[i] -= v;
    return newM;
}


// ------------- //
// -- CLASSES -- //
// ------------- //

class Line
{
    vec3d start;
    vec3d dir;

  public:
    Line()
    {
        start = vec3d{0.0, 0.0, 0.0};
        dir = vec3d{0.0, 0.0, 0.0};
    }

    Line(const vec3d &init_pos, const vec3d &direction)
    {
        start = init_pos;
        dir = glm::normalize(direction);
    }

    vec3d point_at(double t)
    {
        return this->start + t*this->dir;
    }

    vec3d get_start() const
    {
        return this->start;
    }

    vec3d get_direction() const
    {
        return this->dir;
    }
};


class Plane
{
    double d;
    vec3d v1, v2, n;
    mat33d pts;

  public:
    Plane()
    {
        d = -1.0;
        v1 = Z_;
        v2 = X_;
        n = Y_;
        vec3d p0 = {1.0, 1.0, 0.0};
        vec3d p1 = {0.0, 1.0, 1.0};
        vec3d p2 = {0.0, 1.0, 0.0};
        pts = {p0, p1, p2};
    }

    Plane(const mat33d &pts)
    // init from three points
    {
        this->pts = pts;
        v1 = pts[1] - pts[0];
        v2 = pts[2] - pts[0];
        n = glm::normalize(glm::cross(v1, v2));
        d = -glm::dot(n, pts[0]);
    }

    Plane(const vec3d &n, const vec3d &pt)
    // init from normal and point
    {
        this->n = glm::normalize(n);
        pts[0] = pt;
        d = -1.0*glm::dot(n, pts[0]);
        int i = first_non_zero_index(this->n);
        pts[1][i] = -d/this->n[i];
        v1 = pts[1] - pts[0];
        v2 = -glm::cross(v1, this->n);
        pts[2] = pts[0] + v2;
    }

    Plane(const vec3d &pt, const vec3d &v1, const vec3d &v2)
    // init from point and two non-coliniar vectors
    {
        // first check that v1 and v2 are non-colliniar
        assert(!colliniar(v1, v2));

        // now the rest
        pts[0] = pt;
        pts[1] = pt+v1;
        pts[2] = pt+v2;
        this->v1 = v1;
        this->v2 = v2;
        n = glm::normalize(glm::cross(v1, v2));
        d = -1.0*glm::dot(n, pts[0]);
    }

    vec3d get_n() const
    {
        return this->n;
    }

    mat33d get_pts() const
    {
        return this->pts;
    }

    bool point_in_plane(const vec3d &p) const
    {
        return glm::epsilonEqual(glm::dot(p, this->n)+this->d, 0.0, PERCISION);
    }

    bool validate() const
    // run tests to validate all parameters where calculated correctly,
    // irrespective of the constructor used.
    {
        for (int i=0; i<3; i++)
        {
            // test that point p_i conforms to n*p_i =-d
            assert(glm::epsilonEqual(glm::dot(this->pts[i], this->n), -1.0*this->d, PERCISION));
            // test that point p_i is on plane
            assert(this->point_in_plane(this->pts[i]));
        }

        // test that n=v1Xv2/(|v1||v2|)
        assert(glm::all(glm::epsilonEqual(glm::normalize(glm::cross(this->v1, this->v2)), this->n, PERCISION)));

        // test that n is unit length
        assert(glm::epsilonEqual(glm::length2(this->n), 1.0, PERCISION));

        return 1;
    }
};


class Hittable {
    int id;
    int type;
    color3f color;

  public:
    Hittable() {color=WHITE;};
    Hittable(const color3f &c) {color=c;};
    std::string get_color(){
        char buff[100];
        std::sprintf(
            buff, "%0.3f, %0.3f, %0.3f",
            this->color[R], this->color[G], this->color[B]
        );
        std::string sout = buff;
        return sout;
    }

    vec3d reflect(const vec3d &pos, const vec3d &dir) const
    {
        return -Y_;
    }
};


class Sphere: public Hittable
{
    vec3d center;
    double radius;
    double radius2;

  public:
    Sphere()
    {
        center = O3_;
        radius = 1.0;
        radius2 = 1.0;
    }

    Sphere(const vec3d &p, const double &r)
    {
        this->center = p;
        this->radius = r;
        this->radius2 = r*r;
    }

    bool point_on_surface(const vec3d &p) const
    {
        double dist2 = glm::distance2(this->center, p);
        return glm::epsilonEqual(dist2, this->radius2, PERCISION);
    }

    vec3d normal_at_surface(const vec3d &p) const
    {
        // Should check if p on surface?..
        return glm::normalize(p-this->center);
    }

    vec3d reflect(const vec3d &pos, const vec3d &dir) const
    {
        return reflect_ray(dir, this->normal_at_surface(pos));
    }

    vec3d get_center() const
    {
        return this->center;
    }

    double get_r() const
    {
        return this->radius;
    }

    double get_r2() const
    {
        return this->radius2;
    }
};


class Triangle: public Plane, public Hittable
{
  public:
    Triangle(): Plane(), Hittable() {};
    Triangle(const mat33d &pts, const color3f &color): Plane(pts), Hittable(color) {};

    bool point_inside(const vec3d &pt) const
    {
        mat33d trans_pts = this->get_pts()-pt;
        vec3d u = glm::cross(trans_pts[1], trans_pts[2]);
        vec3d v = glm::cross(trans_pts[2], trans_pts[0]);
        vec3d w = glm::cross(trans_pts[0], trans_pts[1]);
        if ( glm::dot(u, v) < 0.0 || glm::dot(u, w) < 0.0 ) return 0;
        return 1;
    }
};


class Screen: public Plane
{
    int w, h;
    double AR, AR_half, AR_inv, AR_inv_half;
    vec2d nw_scs, ne_scs, se_scs, sw_scs, cntr_scs;
    vec3d nw_wcs, ne_wcs, se_wcs, sw_wcs, cntr_wcs;
    mat33d basis_vecs, basis_vecs_inv;
    std::vector<std::vector<color3f>> matrix;

  public:
    Screen(): Plane()
    {
        // resolution related (AR=Aspect Ratio)
        w = 640; h=480;
        AR = (double)w/(double)h;
        AR_half = 0.5*AR;
        AR_inv = 1.0/AR;
        AR_inv_half = 0.5*AR_inv;

        // important points in SCREEN coordinate system
        nw_scs = {0.0, 0.0};
        ne_scs = {1.0, 0.0};
        se_scs = {1.0, AR_inv};
        sw_scs = {0.0, AR_inv};
        cntr_scs = {0.5, AR_inv_half};

        // important points in WORLD coordinate system
        nw_wcs = {-0.5, 1.0, AR_inv_half};
        ne_wcs = {0.5, 1.0, AR_inv_half};
        se_wcs = {0.5, 1.0, -AR_inv_half};
        sw_wcs = {-0.5, 1.0, -AR_inv_half};
        cntr_wcs = {0.0, 1.0, 0.0};

        // screen vectors
        calc_screen_vecs();

        // screen matrix
        std::vector<std::vector<color3f>> matrix2(w, std::vector<color3f>(h));
        matrix = matrix2;
        }

    void get_attributes() const
    {
        std::cout <<
        "Resolution: w=" << this->w << " h=" << this->h << " AR=" << this->AR <<  " AR_inv=" << this->AR_inv << std::endl <<
        "Screen CS points: NW=" << glm::to_string(this->nw_scs) << " NE=" << glm::to_string(this->ne_scs) << " SE=" << glm::to_string(this->se_scs) << " SW=" << glm::to_string(this->sw_scs) << " center=" << glm::to_string(this->cntr_scs) << std::endl <<
        "World CS points: NW=" << glm::to_string(this->nw_wcs) << " NE=" << glm::to_string(this->ne_wcs) << " SE=" << glm::to_string(this->se_wcs) << " SW=" << glm::to_string(this->sw_wcs) << " center=" << glm::to_string(this->cntr_wcs) << std::endl <<
        "Screen vectors: v1=" << glm::to_string(this->basis_vecs[0]) << " v2=" << glm::to_string(this->basis_vecs[1]) << " n=" << glm::to_string(this->basis_vecs[2]) <<
        std::endl;
    }

    void calc_screen_vecs()
    {
        vec3d v1 = glm::normalize(this->se_wcs - this->sw_wcs);
        vec3d v2 = glm::normalize(this->nw_wcs - this->sw_wcs);
        vec3d n = glm::normalize(glm::cross(v2, v1));
        this->basis_vecs = mat33d(v1, v2, n);
        this->basis_vecs_inv = glm::inverse(this->basis_vecs);
        std::cout << glm::to_string(this->basis_vecs*this->basis_vecs_inv) << std::endl;
    }

    vec3d sc_to_wc(const vec2d &sc)
    {

    }

    void rotate_by_point(const double &t, const vec3d &ax, const vec3d &pt)
    {
        this->nw_wcs = rotate_vec_by_pt(this->nw_wcs, t, ax, pt);
        this->ne_wcs = rotate_vec_by_pt(this->ne_wcs, t, ax, pt);
        this->se_wcs = rotate_vec_by_pt(this->se_wcs, t, ax, pt);
        this->sw_wcs = rotate_vec_by_pt(this->sw_wcs, t, ax, pt);
        this->cntr_wcs = rotate_vec_by_pt(this->cntr_wcs, t, ax, pt);
        // re-calculate screen vectors
        this->calc_screen_vecs();
    }

    void rotate_by_center(const double &t, const vec3d &ax)
    {
        this->rotate_by_point(t, ax, this->cntr_wcs);
        this->calc_screen_vecs();
    }

    void translate(const vec3d &dr)
    {
        this->nw_wcs += dr;
        this->ne_wcs += dr;
        this->se_wcs += dr;
        this->sw_wcs += dr;
        this->cntr_wcs += dr;
    }
};


class Camera
{
    vec3d pos, dir;
    Screen screen;

  public:
    Camera()
    {
        pos = O3_;
        dir = Y_;
        Screen screen;
    }

    Camera(const vec3d &pos, const vec3d &dir, const Screen &screen)
    {
        this->pos = pos;
        this->dir = dir;
        this->screen = screen;
    }

    void translate(const vec3d &dx)
    {
        this->pos += dx;
        this->screen.translate(dx);
    }

    void rotate(const double &t, const vec3d &ax)
    {
        this->dir = rotate_vec_by_pt(this->dir, t, ax, this->pos);
        this->screen.rotate_by_point(t, ax, this->pos);
    }

    void zoom(const double &f)
    {
        if ( f <= 0.0 ) return;
        vec3d new_dir = f*this->dir;
        vec3d dx = new_dir - this->dir;
        this->dir = new_dir;
        this->screen.translate(dx);
    }

    Screen* get_screen()
    {
        return &this->screen;
    }
};


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


double line_sphere_intersection(const Line &line, const Sphere &sphere)
{
    vec3d o = line.get_start();
    vec3d u = line.get_direction();
    vec3d c = sphere.get_center();
    double d = glm::dot(o, u-c);
    double D =  d*d - glm::length2(o-c) + sphere.get_r2();
    if ( D <= 0.0 ) return D;
    double sqrtD = glm::sqrt(D);
    double dot_u_oc = -1.0*glm::dot(u, o-c);
    double t1 = dot_u_oc + sqrtD;
    double t2 = dot_u_oc - sqrtD;
    return glm::min(t1, t2);
}


// ---------- //
// -- MAIN -- //
// ---------- //

int main()
{
    // srand(time(NULL));

    Screen screen;

    return 0;
}
