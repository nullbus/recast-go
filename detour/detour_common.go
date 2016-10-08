package detour

import (
	"encoding/binary"
	"io"
	"math"
	"reflect"
	"github.com/go-gl/mathgl/mgl32"
)

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

func min(l, r uint) uint {
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

	return u.endian().Uint32(buffer[:]), nil
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
	case reflect.Int: return 4
	case reflect.Int8 : return 1
	case reflect.Int16 : return 2
	case reflect.Int32 : return 4
	case reflect.Int64: return 8
	case reflect.Uint: return 4
	case reflect.Uint8: return 1
	case reflect.Uint16: return 2
	case reflect.Uint32: return 4
	case reflect.Uint64: return 8
	case reflect.Float32: return 4
	case reflect.Float64: return 8
	case reflect.Array:
		return sizeof(t.Elem()) * t.Len()
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

func minv(l, r mgl32.Vec3) {
	mgl32.SetMin(&l[0], &r[0])
	mgl32.SetMin(&l[1], &r[1])
	mgl32.SetMin(&l[2], &r[2])
}

func maxv(l, r mgl32.Vec3) {
	mgl32.SetMax(&l[0], &r[0])
	mgl32.SetMax(&l[1], &r[1])
	mgl32.SetMax(&l[2], &r[2])
}