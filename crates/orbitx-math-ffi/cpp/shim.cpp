// Shim that wraps Orbiter's Vecmat/Astro functions behind a stable extern "C"
// ABI for Rust FFI. Compiled by build.rs which copies Orbiter's Vecmat/Astro
// sources into OUT_DIR (patching the friend-default-argument clang rejects)
// and forces `orbiter_stub.h` to replace `OrbiterAPI.h`.
//
// IMPORTANT: all struct arguments and returns go through raw pointers to avoid
// ABI drift between clang's union/struct-by-value calling convention and
// Rust's #[repr(C)] struct convention. The Rust side constructs values in
// #[repr(C)] shadow structs with identical byte layouts and passes pointers.

#include <cstring>

#include "orbiter_stub.h"
#include "Vecmat.h"
#include "Astro.h"

// --- Vec3 wrappers ---

extern "C" double ox_crossp(const Vector* a, const Vector* b, Vector* out) {
    *out = crossp(*a, *b);
    return 0.0;
}
extern "C" double ox_dotp(const Vector* a, const Vector* b) {
    return dotp(*a, *b);
}
extern "C" double ox_v3_length2(const Vector* v) {
    return v->length2();
}
extern "C" double ox_v3_unit(const Vector* v, Vector* out) {
    *out = v->unit();
    return 0.0;
}
extern "C" double ox_v3_dist2(const Vector* a, const Vector* b) {
    return a->dist2(*b);
}
extern "C" double ox_xangle(const Vector* a, const Vector* b) {
    return xangle(*a, *b);
}
extern "C" void ox_v3_add(const Vector* a, const Vector* b, Vector* out) { *out = *a + *b; }
extern "C" void ox_v3_sub(const Vector* a, const Vector* b, Vector* out) { *out = *a - *b; }
extern "C" void ox_v3_mul_scalar(const Vector* a, double s, Vector* out) { *out = *a * s; }
extern "C" void ox_v3_hadamard(const Vector* a, const Vector* b, Vector* out) { *out = *a * *b; }

// --- Matrix3 wrappers ---

extern "C" void ox_m3_mul_m(const Matrix* a, const Matrix* b, Matrix* out) { *out = *a * *b; }
extern "C" void ox_mul(const Matrix* a, const Vector* b, Vector* out) { *out = mul(*a, *b); }
extern "C" void ox_tmul(const Matrix* a, const Vector* b, Vector* out) { *out = tmul(*a, *b); }
extern "C" void ox_inv(const Matrix* a, Matrix* out) { *out = inv(*a); }
extern "C" void ox_transp(const Matrix* a, Matrix* out) { *out = transp(*a); }
extern "C" void ox_imatrix(Matrix* out) { *out = IMatrix(); }

extern "C" void ox_m3_from_quat(const Quaternion* q, Matrix* out) {
    out->Set(*q);
}

extern "C" void ox_m3_from_euler(const Vector* rot, Matrix* out) {
    out->Set(*rot);
}

extern "C" void ox_orthogonalise(Matrix* m, int axis) {
    m->orthogonalise(axis);
}

extern "C" void ox_qrdcmp3(Matrix* a, Vector* c, Vector* d, int* sing) {
    int s = 0;
    qrdcmp(*a, *c, *d, &s);
    if (sing) *sing = s;
}

extern "C" void ox_qrsolv3(const Matrix* a, const Vector* c, const Vector* d, Vector* b) {
    qrsolv(*a, *c, *d, *b);
}

// --- Quaternion wrappers ---

extern "C" void ox_q_identity(Quaternion* out) {
    Quaternion q;
    *out = q;
}

extern "C" void ox_q_from_matrix(const Matrix* r, Quaternion* out) {
    Quaternion q(*r);
    *out = q;
}

extern "C" void ox_q_hamilton(const Quaternion* a, const Quaternion* b, Quaternion* out) {
    *out = *a * *b;
}

extern "C" void ox_q_mul_vec(const Quaternion* q, const Vector* p, Vector* out) {
    *out = mul(*q, *p);
}

extern "C" void ox_q_tmul_vec(const Quaternion* q, const Vector* p, Vector* out) {
    *out = tmul(*q, *p);
}

extern "C" void ox_q_rotate(const Quaternion* q, const Vector* omega, Quaternion* out) {
    Quaternion r = *q;
    r.Rotate(*omega);
    *out = r;
}

extern "C" void ox_q_interp(const Quaternion* a, const Quaternion* b, double u, Quaternion* out) {
    Quaternion r;
    r.interp(*a, *b, u);
    *out = r;
}

extern "C" double ox_q_norm2(const Quaternion* q) {
    return q->norm2();
}

// --- Geometry wrappers ---

extern "C" void ox_plane_coeffs(const Vector* p1, const Vector* p2, const Vector* p3,
                                 double* a, double* b, double* c, double* d) {
    PlaneCoeffs(*p1, *p2, *p3, *a, *b, *c, *d);
}

extern "C" double ox_point_line_dist(const Vector* a, const Vector* p, const Vector* d) {
    return PointLineDist(*a, *p, *d);
}

extern "C" double ox_point_plane_dist(const Vector* p, double a, double b, double c, double d) {
    return PointPlaneDist(*p, a, b, c, d);
}

extern "C" void ox_vector_basis_to_matrix(const Vector* x, const Vector* y, const Vector* z, Matrix* r) {
    VectorBasisToMatrix(*x, *y, *z, *r);
}

extern "C" void ox_dir_rot_to_matrix(const Vector* z, const Vector* y, Matrix* r) {
    DirRotToMatrix(*z, *y, *r);
}

// --- Astro wrappers ---

extern "C" double ox_obliquity(double jc) { return Obliquity(jc); }

extern "C" void ox_equ2ecl(double cosob, double sinob, double ra, double dc,
                            double* l, double* b) {
    Equ2Ecl(cosob, sinob, ra, dc, *l, *b);
}

extern "C" void ox_ecl2equ(double cosob, double sinob, double l, double b,
                            double* ra, double* dc) {
    Ecl2Equ(cosob, sinob, l, b, *ra, *dc);
}

extern "C" double ox_orthodrome_dist(double lng1, double lat1, double lng2, double lat2) {
    return Orthodome(lng1, lat1, lng2, lat2);
}

extern "C" void ox_orthodrome(double lng1, double lat1, double lng2, double lat2,
                               double* dist, double* dir) {
    Orthodome(lng1, lat1, lng2, lat2, *dist, *dir);
}

// date_to_mjd via a plain POD struct matching the Rust COxDate layout.
struct OxDate { int year, month, day, hour, min, sec; };

extern "C" double ox_date_to_mjd(const OxDate* d) {
    struct tm t;
    std::memset(&t, 0, sizeof(t));
    t.tm_year = d->year - 1900;
    // NOTE: Orbiter's date2mjd treats tm_mon as 1-12 (not standard C 0-11),
    // matching mjddate's output convention. Pass month through directly.
    t.tm_mon  = d->month;
    t.tm_mday = d->day;
    t.tm_hour = d->hour;
    t.tm_min  = d->min;
    t.tm_sec  = d->sec;
    return date2mjd(&t);
}
