// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class, 3D math helper
// 6/5/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2012-06-05 - add 3D math helper file to DMP6 example sketch

/* ============================================
ChibiOS I2Cdev MPU6050 I2C device class, math helper code is placed under the MIT license
Copyright (c) 2012 Jan Schlemminger

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#ifndef _HELPER_3DMATH_H_
#define _HELPER_3DMATH_H_

#include "ch.h"
#include "math.h"

typedef struct {
        float w;
        float x;
        float y;
        float z;
} Quaternion;

typedef struct {
        int16_t x;
        int16_t y;
        int16_t z;
} VectorInt16;

typedef struct {
        float x;
        float y;
        float z;
} VectorFloat;

static Quaternion getProduct(const Quaternion* q1, const Quaternion* q2) {
		// Quaternion multiplication is defined by:
		//     (Q1 * Q2).w = (w1w2 - x1x2 - y1y2 - z1z2)
		//     (Q1 * Q2).x = (w1x2 + x1w2 + y1z2 - z1y2)
		//     (Q1 * Q2).y = (w1y2 - x1z2 + y1w2 + z1x2)
		//     (Q1 * Q2).z = (w1z2 + x1y2 - y1x2 + z1w2
		Quaternion tmp;
		tmp.w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z; 
		tmp.x = q1->w * q2->x - q1->x * q2->w - q1->y * q2->z - q1->z * q2->y; 
		tmp.y = q1->w * q2->y - q1->x * q2->z - q1->y * q2->w - q1->z * q2->x; 
		tmp.z = q1->w * q2->z - q1->x * q2->y - q1->y * q2->x - q1->z * q2->w; 
		return tmp;
}

static Quaternion getConjugate(const Quaternion* q) {
		Quaternion tmp;
		tmp.w = q->w;
		tmp.x = -q->x;
		tmp.y = -q->y;
		tmp.z = -q->z;
		return tmp;
}
        
static float getMagnitudeQuat(const Quaternion* q) {
		return sqrtf(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);
}
        
static void normalizeQuat(Quaternion* q) {
		float m = getMagnitudeQuat(q);
		q->w /= m;
		q->x /= m;
		q->y /= m;
		q->z /= m;
}

static float getMagnitudeVectInt(const VectorInt16* v) {
		return sqrt(v->x*v->x + v->y*v->y + v->z*v->z);
}

static void normalizeVectInt(VectorInt16* v) {
		float m = getMagnitudeVectInt(v);
		v->x /= m;
		v->y /= m;
		v->z /= m;
}

static void rotateVectInt(VectorInt16* v, Quaternion *q) {
		// http://www.cprogramming.com/tutorial/3d/quaternions.html
		// http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
		// http://content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation
		// ^ or: http://webcache.googleusercontent.com/search?q=cache:xgJAp3bDNhQJ:content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation&hl=en&gl=us&strip=1

		// P_out = q * P_in * conj(q)
		// - P_out is the output vector
		// - q is the orientation quaternion
		// - P_in is the input vector (a*aReal)
		// - conj(q) is the conjugate of the orientation quaternion (q=[w,x,y,z], q*=[w,-x,-y,-z])
		Quaternion tmpConjugate;	
		
		Quaternion p;
		p.w = 0;
		p.x = v->x;
		p.y = v->y;
		p.z = v->z;	

		// quaternion multiplication: q * p, stored back in p
		p = getProduct(q, &p);

		// quaternion multiplication: p * conj(q), stored back in p
		tmpConjugate = getConjugate(q);
		p = getProduct(&p, &tmpConjugate);

		// p quaternion is now [0, x', y', z']
		v->x = p.x;
		v->y = p.y;
		v->z = p.z;
}

static float getMagnitudeVectFloat(const VectorFloat* v) {
		return sqrtf(v->x*v->x + v->y*v->y + v->z*v->z);
}

static void normalizeVectFloat(VectorFloat* v) {
		float m = getMagnitudeVectFloat(v);
		v->x /= m;
		v->y /= m;
		v->z /= m;
}
        
static void rotateVectFloat(VectorFloat* v, Quaternion *q) {
		Quaternion tmpConjugate;	
		
		Quaternion p;
		p.w = 0;
		p.x = v->x;
		p.y = v->y;
		p.z = v->z;	

		// quaternion multiplication: q * p, stored back in p
		p = getProduct(q, &p);

		// quaternion multiplication: p * conj(q), stored back in p
		tmpConjugate = getConjugate(q);
		p = getProduct(&p, &tmpConjugate);

		// p quaternion is now [0, x', y', z']
		v->x = p.x;
		v->y = p.y;
		v->z = p.z;
}

#endif /* _HELPER_3DMATH_H_ */
