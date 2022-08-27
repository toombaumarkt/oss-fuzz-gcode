
#ifndef WRAPPER_H
#define WRAPPER_H

#include "Eigen/Geometry"
#include <vector>

// fuzzing parameters
#define MAX_DATA_SIZE 1024
#define MIN_DATA_SIZE 1

#define DEBUG_OUTPUT 0


// copied from ShinyMacros.h
#define PROFILE_FUNC()
#define PROFILE_BLOCK(name)


// copied from LocalesUtils.cpp
inline bool is_decimal_separator_point()
{
    char str[5] = "";
    sprintf(str, "%.1f", 0.5f);
    return str[1] == '.';
}


// copied from types.h
typedef signed char __int8_t;
typedef unsigned char __uint8_t;
typedef signed short int __int16_t;
typedef unsigned short int __uint16_t;
typedef signed int __int32_t;
typedef unsigned int __uint32_t;
#if __WORDSIZE == 64
typedef signed long int __int64_t;
typedef unsigned long int __uint64_t;
#else
__extension__ typedef signed long long int __int64_t;
__extension__ typedef unsigned long long int __uint64_t;
#endif

typedef __int8_t int8_t;
typedef __int16_t int16_t;
typedef __int32_t int32_t;
typedef __int64_t int64_t;


// copied from libslic3r.h
using coordf_t = double;
#if 1
// Saves around 32% RAM after slicing step, 6.7% after G-code export (tested on PrusaSlicer 2.2.0 final).
using coord_t = int32_t;
#else
//FIXME At least FillRectilinear2 and std::boost Voronoi require coord_t to be 32bit.
typedef int64_t coord_t;
#endif


// copied from libslic3er.h
enum Axis { 
	X=0,
	Y,
	Z,
	E,
	F,
	NUM_AXES,
	// For the GCodeReader to mark a parsed axis, which is not in "XYZEF", it was parsed correctly.
	UNKNOWN_AXIS = NUM_AXES,
	NUM_AXES_WITH_UNKNOWN,
};
static constexpr double SCALING_FACTOR = 0.000001;
#define scale_(val) ((val) / SCALING_FACTOR)


class Line;
class MultiPoint;
class Point;


// copied from Point.hpp & Line.hpp
using Points         = std::vector<Point>;
using PointPtrs      = std::vector<Point*>;
using PointConstPtrs = std::vector<const Point*>;
using Vec2crd = Eigen::Matrix<coord_t,  2, 1, Eigen::DontAlign>;
using Vec2d   = Eigen::Matrix<double,   2, 1, Eigen::DontAlign>;
using Vec2f   = Eigen::Matrix<float,    2, 1, Eigen::DontAlign>;
typedef std::vector<Line> Lines;

class Point : public Vec2crd
{
public:
    using coord_type = coord_t;

    Point() : Vec2crd(0, 0) {}
    Point(int32_t x, int32_t y) : Vec2crd(coord_t(x), coord_t(y)) {}
    Point(int64_t x, int64_t y) : Vec2crd(coord_t(x), coord_t(y)) {}
    Point(double x, double y) : Vec2crd(coord_t(lrint(x)), coord_t(lrint(y))) {}
    Point(const Point &rhs) { *this = rhs; }
	explicit Point(const Vec2d& rhs) : Vec2crd(coord_t(lrint(rhs.x())), coord_t(lrint(rhs.y()))) {}
	// This constructor allows you to construct Point from Eigen expressions
    template<typename OtherDerived>
    Point(const Eigen::MatrixBase<OtherDerived> &other) : Vec2crd(other) {}
    static Point new_scale(coordf_t x, coordf_t y) { return Point(coord_t(scale_(x)), coord_t(scale_(y))); }
    static Point new_scale(const Vec2d &v) { return Point(coord_t(scale_(v.x())), coord_t(scale_(v.y()))); }
    static Point new_scale(const Vec2f &v) { return Point(coord_t(scale_(v.x())), coord_t(scale_(v.y()))); }

    // This method allows you to assign Eigen expressions to MyVectorType
    template<typename OtherDerived>
    Point& operator=(const Eigen::MatrixBase<OtherDerived> &other)
    {
        this->Vec2crd::operator=(other);
        return *this;
    }

    Point& operator+=(const Point& rhs) { this->x() += rhs.x(); this->y() += rhs.y(); return *this; }
    Point& operator-=(const Point& rhs) { this->x() -= rhs.x(); this->y() -= rhs.y(); return *this; }
	Point& operator*=(const double &rhs) { this->x() = coord_t(this->x() * rhs); this->y() = coord_t(this->y() * rhs); return *this; }
    Point operator*(const double &rhs) { return Point(this->x() * rhs, this->y() * rhs); }

    void   rotate(double angle) { this->rotate(std::cos(angle), std::sin(angle)); }
    void   rotate(double cos_a, double sin_a) {
        double cur_x = (double)this->x();
        double cur_y = (double)this->y();
        this->x() = (coord_t)round(cos_a * cur_x - sin_a * cur_y);
        this->y() = (coord_t)round(cos_a * cur_y + sin_a * cur_x);
    }

    void   rotate(double angle, const Point &center);
    Point  rotated(double angle) const { Point res(*this); res.rotate(angle); return res; }
    Point  rotated(double cos_a, double sin_a) const { Point res(*this); res.rotate(cos_a, sin_a); return res; }
    Point  rotated(double angle, const Point &center) const { Point res(*this); res.rotate(angle, center); return res; }
    int    nearest_point_index(const Points &points) const;
    int    nearest_point_index(const PointConstPtrs &points) const;
    int    nearest_point_index(const PointPtrs &points) const;
    bool   nearest_point(const Points &points, Point* point) const;
    Point  projection_onto(const MultiPoint &poly) const;
    Point  projection_onto(const Line &line) const;
};



// copied from MultiPoint.h
class MultiPoint
{
public:
    Points points;
    
    MultiPoint() = default;
    virtual ~MultiPoint() = default;
    MultiPoint(const MultiPoint &other) : points(other.points) {}
    MultiPoint(MultiPoint &&other) : points(std::move(other.points)) {}
    MultiPoint(std::initializer_list<Point> list) : points(list) {}
    explicit MultiPoint(const Points &_points) : points(_points) {}
    MultiPoint& operator=(const MultiPoint &other) { points = other.points; return *this; }
    MultiPoint& operator=(MultiPoint &&other) { points = std::move(other.points); return *this; }
    void scale(double factor);
    void scale(double factor_x, double factor_y);
    void translate(double x, double y) { this->translate(Point(coord_t(x), coord_t(y))); }
    void translate(const Point &vector);
    void rotate(double angle) { this->rotate(cos(angle), sin(angle)); }
    void rotate(double cos_angle, double sin_angle);
    void rotate(double angle, const Point &center);
    void reverse() { std::reverse(this->points.begin(), this->points.end()); }

    const Point& front() const { return this->points.front(); }
    const Point& back() const { return this->points.back(); }
    const Point& first_point() const { return this->front(); }
    virtual const Point& last_point() const = 0;
    virtual Lines lines() const = 0;
    size_t size() const { return points.size(); }
    bool   empty() const { return points.empty(); }
    double length() const;
    bool   is_valid() const { return this->points.size() >= 2; }

    int  find_point(const Point &point) const;
    bool has_boundary_point(const Point &point) const;
    int  closest_point_index(const Point &point) const {
        int idx = -1;
        if (! this->points.empty()) {
            idx = 0;
            double dist_min = (point - this->points.front()).cast<double>().norm();
            for (int i = 1; i < int(this->points.size()); ++ i) {
                double d = (this->points[i] - point).cast<double>().norm();
                if (d < dist_min) {
                    dist_min = d;
                    idx = i;
                }
            }
        }
        return idx;
    }
    const Point* closest_point(const Point &point) const { return this->points.empty() ? nullptr : &this->points[this->closest_point_index(point)]; }
    // BoundingBox bounding_box() const;
    // Return true if there are exact duplicates.
    bool has_duplicate_points() const;
    // Remove exact duplicates, return true if any duplicate has been removed.
    bool remove_duplicate_points();
    void clear() { this->points.clear(); }
    void append(const Point &point) { this->points.push_back(point); }
    void append(const Points &src) { this->append(src.begin(), src.end()); }
    void append(const Points::const_iterator &begin, const Points::const_iterator &end) { this->points.insert(this->points.end(), begin, end); }
    // void append(Points &&src)
    // {
    //     if (this->points.empty()) {
    //         this->points = std::move(src);
    //     } else {
    //         this->points.insert(this->points.end(), src.begin(), src.end());
    //         src.clear();
    //     }
    // }

    bool intersection(const Line& line, Point* intersection) const;
    bool first_intersection(const Line& line, Point* intersection) const;
    bool intersections(const Line &line, Points *intersections) const;

    static Points _douglas_peucker(const Points &points, const double tolerance);
    static Points visivalingam(const Points& pts, const double& tolerance);

    inline auto begin()        { return points.begin(); }
    inline auto begin()  const { return points.begin(); }
    inline auto end()          { return points.end();   }
    inline auto end()    const { return points.end();   }
    inline auto cbegin() const { return points.begin(); }
    inline auto cend()   const { return points.end();   }
};

// copied from Line.hpp

using Vector = Point;
class Line
{
public:
    Line() {}
    Line(const Point& _a, const Point& _b) : a(_a), b(_b) {}
    explicit operator Lines() const { Lines lines; lines.emplace_back(*this); return lines; }
    void   scale(double factor) { this->a *= factor; this->b *= factor; }
    void   translate(const Point &v) { this->a += v; this->b += v; }
    void   translate(double x, double y) { this->translate(Point(x, y)); }
    void   rotate(double angle, const Point &center) { this->a.rotate(angle, center); this->b.rotate(angle, center); }
    void   reverse() { std::swap(this->a, this->b); }
    double length() const { return (b - a).cast<double>().norm(); }
    Point  midpoint() const { return (this->a + this->b) / 2; }
    bool   intersection_infinite(const Line &other, Point* point) const;
    bool   operator==(const Line &rhs) const { return this->a == rhs.a && this->b == rhs.b; }
    // double distance_to_squared(const Point &point) const { return distance_to_squared(point, this->a, this->b); }
    // double distance_to_squared(const Point &point, Point *closest_point) const { return line_alg::distance_to_squared(*this, point, closest_point); }
    // double distance_to(const Point &point) const { return distance_to(point, this->a, this->b); }
    double perp_distance_to(const Point &point) const;
    bool   parallel_to(double angle) const;
    bool   parallel_to(const Line& line) const;
    bool   perpendicular_to(double angle) const;
    bool   perpendicular_to(const Line& line) const;
    double atan2_() const { return atan2(this->b(1) - this->a(1), this->b(0) - this->a(0)); }
    double orientation() const;
    double direction() const;
    Vector vector() const { return this->b - this->a; }
    Vector normal() const { return Vector((this->b(1) - this->a(1)), -(this->b(0) - this->a(0))); }
    bool   intersection(const Line& line, Point* intersection) const;
    // Clip a line with a bounding box. Returns false if the line is completely outside of the bounding box.
	// bool   clip_with_bbox(const BoundingBox &bbox);
    // Extend the line from both sides by an offset.
    void   extend(double offset);

    // static inline double distance_to_squared(const Point &point, const Point &a, const Point &b) { return line_alg::distance_to_squared(Line{a, b}, Vec<2, coord_t>{point}); }
    // static double distance_to(const Point &point, const Point &a, const Point &b) { return sqrt(distance_to_squared(point, a, b)); }

    Point a;
    Point b;

    static const constexpr int Dim = 2;
    using Scalar = Point::Scalar;
};


// copied from Utils.hpp
struct FilePtr {
    FilePtr(FILE *f) : f(f) {}
    ~FilePtr() { this->close(); }
    void close() { 
        if (this->f) {
            ::fclose(this->f);
            this->f = nullptr;
        }
    }
    FILE* f = nullptr;
};


#endif
