package detour

import (
	"encoding/binary"
	"io"
	"math"
	"reflect"
)

type Vector3 []float32

func NewVector3(list []float32) Vector3 {
	return list[:3]
}

func (v Vector3) CopyFrom(src Vector3) {
	copy(v[:3], src[:3])
}

func (v1 Vector3) Add(v2 Vector3) Vector3 {
	return Vector3{v1[0] + v2[0], v1[1] + v2[1], v1[2] + v2[2]}
}

func (v1 Vector3) Sub(v2 Vector3) Vector3 {
	return Vector3{v1[0] - v2[0], v1[1] - v2[1], v1[2] - v2[2]}
}

func (v1 Vector3) Dot(v2 Vector3) float32 {
	return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]
}

// ignore Y values
func (v1 Vector3) Dot2D(v2 Vector3) float32 {
	return v1[0]*v2[0] + v1[2]*v2[2]
}

func (v Vector3) Len() float32 {
	return float32(math.Sqrt(float64(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])))
}

func (v Vector3) DistanceTo(target Vector3) float32 {
	return target.Sub(v).Len()
}

func (v Vector3) X() float32 {
	return v[0]
}

func (v Vector3) Y() float32 {
	return v[1]
}

func (v Vector3) Z() float32 {
	return v[2]
}

func align4(x int) int {
	return (x + 3) & ^3
}

func nextPow2(v uint) uint {
	v--
	v |= v >> 1
	v |= v >> 2
	v |= v >> 4
	v |= v >> 8
	v |= v >> 16
	v++
	return v
}

func ilog2(v uint) uint {
	var r, shift1, shift2, shift3 uint

	if v > 0xffff {
		r = 1 << 4
	}
	v >>= r

	if v > 0xff {
		shift1 = 1 << 3
	}
	v >>= shift1
	r |= shift1

	if v > 0xf {
		shift2 = 1 << 2
	}
	v >>= shift2
	r |= shift2

	if v > 0x3 {
		shift3 = 1 << 1
	}
	v >>= shift3
	r |= shift3

	r |= (v >> 1)
	return r
}

func max(l, r uint) uint {
	if l < r {
		return r
	}

	return l
}

func min(l, r uint) uint {
	if l > r {
		return r
	}

	return l
}

func fmax(l, r float32) float32 {
	if l < r {
		return r
	}

	return l
}

func fmin(l, r float32) float32 {
	if l > r {
		return r
	}

	return l
}

type UtilReader struct {
	r      io.Reader
	Endian binary.ByteOrder
}

func (u *UtilReader) endian() binary.ByteOrder {
	if u.Endian == nil {
		return binary.LittleEndian
	}

	return u.Endian
}

func (u *UtilReader) ReadInt() (int, error) {
	var buffer [4]byte
	if _, err := u.r.Read(buffer[:]); err != nil {
		return 0, err
	}

	return int(u.endian().Uint32(buffer[:])), nil
}

func (u *UtilReader) ReadUint() (uint, error) {
	var buffer [4]byte
	if _, err := u.r.Read(buffer[:]); err != nil {
		return 0, err
	}

	return uint(u.endian().Uint32(buffer[:])), nil
}

func (u *UtilReader) ReadFloat() (float32, error) {
	var buffer [4]byte
	if _, err := u.r.Read(buffer[:]); err != nil {
		return 0, err
	}

	return math.Float32frombits(u.endian().Uint32(buffer[:])), nil
}

func (u *UtilReader) ReadDouble() (float64, error) {
	var buffer [8]byte
	if _, err := u.r.Read(buffer[:]); err != nil {
		return 0, err
	}

	return math.Float64frombits(u.endian().Uint64(buffer[:])), nil
}

func sizeof(obj interface{}) int {
	t, ok := obj.(reflect.Type)
	if !ok {
		t = reflect.TypeOf(&obj).Elem()
	}
	switch t.Kind() {
	case reflect.Int:
		return 4
	case reflect.Int8:
		return 1
	case reflect.Int16:
		return 2
	case reflect.Int32:
		return 4
	case reflect.Int64:
		return 8
	case reflect.Uint:
		return 4
	case reflect.Uint8:
		return 1
	case reflect.Uint16:
		return 2
	case reflect.Uint32:
		return 4
	case reflect.Uint64:
		return 8
	case reflect.Float32:
		return 4
	case reflect.Float64:
		return 8
	case reflect.Array:
		return sizeof(t.Elem()) * t.Len()
	case reflect.Struct:
		var sum int
		for i := 0; i < t.NumField(); i++ {
			sum += sizeof(t.Field(i).Type)
		}
		return sum
	}

	panic("unsupported type")
}

func clamp(v, min, max float32) float32 {
	if v < min {
		return min
	}
	if v > max {
		return max
	}
	return v
}

func vmin(l, r Vector3) {
	if l[0] > r[0] {
		l[0] = r[0]
	}
	if l[1] > r[1] {
		l[1] = r[1]
	}
	if l[2] > r[2] {
		l[2] = r[2]
	}
}

func vmax(l, r Vector3) {
	if l[0] < r[0] {
		l[0] = r[0]
	}
	if l[1] < r[1] {
		l[1] = r[1]
	}
	if l[2] < r[2] {
		l[2] = r[2]
	}
}

/// Performs a linear interpolation between two vectors. (@p v1 toward @p v2)
///  @param[out]	dest	The result vector. [(x, y, x)]
///  @param[in]		v1		The starting vector.
///  @param[in]		v2		The destination vector.
///	 @param[in]		t		The interpolation factor. [Limits: 0 <= value <= 1.0]
func vlerp(v1, v2 Vector3, t float32) (dest Vector3) {
	return Vector3{
		v1[0] + (v2[0]-v1[0])*t,
		v1[1] + (v2[1]-v1[1])*t,
		v1[2] + (v2[2]-v1[2])*t,
	}
}

func abs(v float32) float32 {
	if v > 0 {
		return v
	}

	return -v
}

func sqr(v float32) float32 {
	return v * v
}

func sqrt(v float32) float32 {
	return float32(math.Sqrt(float64(v)))
}

/// Determines if two axis-aligned bounding boxes overlap.
///  @param[in]		amin	Minimum bounds of box A. [(x, y, z)]
///  @param[in]		amax	Maximum bounds of box A. [(x, y, z)]
///  @param[in]		bmin	Minimum bounds of box B. [(x, y, z)]
///  @param[in]		bmax	Maximum bounds of box B. [(x, y, z)]
/// @return True if the two AABB's overlap.
/// @see dtOverlapBounds
func overlapQuantBounds(amin, amax, bmin, bmax [3]uint16) bool {
	overlap := true
	if amin[0] > bmax[0] || amax[0] < bmin[0] {
		overlap = false
	}
	if amin[1] > bmax[1] || amax[1] < bmin[1] {
		overlap = false
	}
	if amin[2] > bmax[2] || amax[2] < bmin[2] {
		overlap = false
	}
	return overlap
}

/// Determines if two axis-aligned bounding boxes overlap.
///  @param[in]		amin	Minimum bounds of box A. [(x, y, z)]
///  @param[in]		amax	Maximum bounds of box A. [(x, y, z)]
///  @param[in]		bmin	Minimum bounds of box B. [(x, y, z)]
///  @param[in]		bmax	Maximum bounds of box B. [(x, y, z)]
/// @return True if the two AABB's overlap.
/// @see dtOverlapQuantBounds
func overlapBounds(amin, amax, bmin, bmax Vector3) bool {
	overlap := true
	if amin[0] > bmax[0] || amax[0] < bmin[0] {
		overlap = false
	}
	if amin[1] > bmax[1] || amax[1] < bmin[1] {
		overlap = false
	}
	if amin[2] > bmax[2] || amax[2] < bmin[2] {
		overlap = false
	}
	return overlap
}

func oppositeTile(side int) int {
	return (side + 4) & 0x7
}

func closestHeightPointTriangle(p, a, b, c Vector3) (h float32, ok bool) {
	v0 := c.Sub(a)
	v1 := b.Sub(a)
	v2 := p.Sub(a)

	dot00 := v0.Dot2D(v0)
	dot01 := v0.Dot2D(v1)
	dot02 := v0.Dot2D(v2)
	dot11 := v1.Dot2D(v1)
	dot12 := v1.Dot2D(v2)

	// Compute barycentric coordinates
	invDenom := 1 / (dot00*dot11 - dot01*dot01)
	u := (dot11*dot02 - dot01*dot12) * invDenom
	v := (dot00*dot12 - dot01*dot02) * invDenom

	// The (sloppy) epsilon is needed to allow to get height of points which
	// are interpolated along the edges of the triangles.
	const EPS = 1e-4

	// If point lies inside the triangle, return interpolated ycoord.
	if u >= -EPS && v >= -EPS && (u+v) <= 1+EPS {
		return a[1] + v0[1]*u + v1[1]*v, true
	}

	return 0, false
}

func distancePtPolyEdgesSqr(pt Vector3, verts []Vector3, nverts int, ed, et []float32) bool {
	// TODO: Replace pnpoly with triArea2D tests?
	c := false
	for i, j := 0, nverts-1; i < nverts; j, i = i, i+1 {
		vi := verts[i]
		vj := verts[j]
		if ((vi[2] > pt[2]) != (vj[2] > pt[2])) && (pt[0] < (vj[0]-vi[0])*(pt[2]-vi[2])/(vj[2]-vi[2])+vi[0]) {
			c = !c
		}
		et[j], ed[j] = distancePtSegSqr2D(pt, vj, vi)
	}
	return c
}

func distancePtSegSqr2D(pt, p, q Vector3) (float32, float32) {
	pqx := q[0] - p[0]
	pqz := q[2] - p[2]
	dx := pt[0] - p[0]
	dz := pt[2] - p[2]
	d := pqx*pqx + pqz*pqz
	t := pqx*dx + pqz*dz
	if d > 0 {
		t /= d
	}
	if t < 0 {
		t = 0
	} else if t > 1 {
		t = 1
	}
	dx = p[0] + t*pqx - pt[0]
	dz = p[2] + t*pqz - pt[2]
	return t, dx*dx + dz*dz
}
