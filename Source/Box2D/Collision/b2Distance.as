/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

package Box2D.Collision{
	
import Box2D.Common.Math.*;
import Box2D.Common.*;
import Box2D.Collision.Shapes.*;
import Box2D.Collision.*;

import Box2D.Common.b2internal;
use namespace b2internal;

/**
* @private
*/
public class b2Distance
{

// GJK using Voronoi regions (Christer Ericson) and Barycentric coordinates.

private static var b2_gjkCalls:int;
private static var b2_gjkIters:int;
private static var b2_gjkMaxIters:int;

public static function Distance(output:b2DistanceOutput, cache:b2SimplexCache, input:b2DistanceInput):void
{
	++b2_gjkCalls;
	
	var proxyA:b2DistanceProxy = input.proxyA;
	var proxyB:b2DistanceProxy = input.proxyB;
	
	var transformA:b2Transform = input.transformA;
	var transformB:b2Transform = input.transformB;
	
	// Initialize the simplex
	var simplex:b2Simplex = new b2Simplex();
	simplex.ReadCache(cache, proxyA, transformA, proxyB, transformB);
	
	// Get simplex vertices as an array.
	var vertices:Array/*b2SimplexVertex*/= simplex.m_vertices;
	const k_maxIters:int = 20;
	
	// These store the vertices of the last simplex so that we
	// can check for duplicates and preven cycling
	var saveA:Array/*int*/ = new Array(3);
	var saveB:Array/*int*/ = new Array(3);
	var saveCount:int = 0;
	
	var closestPoint:b2Vec2 = simplex.GetClosestPoint();
	var distanceSqr1:Number = closestPoint.LengthSquared();
	var distanceSqr2:Number = distanceSqr1;
	
	var i:int;
	var p:b2Vec2;
	
	// Main iteration loop
	var iter:int = 0;
	while (iter < k_maxIters)
	{
		// Copy the simplex so that we can identify duplicates
		saveCount = simplex.m_count;
		for (i = 0; i < saveCount; i++)
		{
			saveA[i] = vertices[i].indexA;
			saveB[i] = vertices[i].indexB;
		}
		
		switch(simplex.m_count)
		{
			case 1:
				break;
			case 2:
				simplex.Solve2();
				break;
			case 3:
				simplex.Solve3();
				break;
			default:
				b2Settings.b2Assert(false);
		}
		
		// If we have 3 points, then the origin is in the corresponding triangle.
		if (simplex.m_count == 3)
		{
			break;
		}
		
		// Compute the closest point.
		p = simplex.GetClosestPoint();
		distanceSqr2 = p.LengthSquared();
		
		// Ensure progress
		if (distanceSqr2 > distanceSqr1)
		{
			//break;
		}
		distanceSqr1 = distanceSqr2;
		
		// Get search direction.
		var d:b2Vec2 = simplex.GetSearchDirection();
		
		// Ensure the search direction is numerically fit.
		if (d.LengthSquared() < Number.MIN_VALUE * Number.MIN_VALUE)
		{
			// THe origin is probably contained by a line segment or triangle.
			// Thus the shapes are overlapped.
			
			// We can't return zero here even though there may be overlap.
			// In case the simplex is a point, segment or triangle it is very difficult
			// to determine if the origin is contained in the CSO or very close to it
			break;
		}
		
		// Compute a tentative new simplex vertex using support points
		var vertex:b2SimplexVertex = vertices[simplex.m_count];
		vertex.indexA = proxyA.GetSupport(b2Math.b2MulTMV(transformA.R, d.Negative()));
		vertex.wA = b2Math.b2MulX(transformA, proxyA.GetVertex(vertex.indexA));
		vertex.indexB = proxyB.GetSupport(b2Math.b2MulTMV(transformB.R, d));
		vertex.wB = b2Math.b2MulX(transformB, proxyB.GetVertex(vertex.indexB));
		vertex.w = b2Math.SubtractVV(vertex.wB, vertex.wA);
		
		// Iteration count is equated to the number of support point calls.
		++iter;
		++b2_gjkIters;
		
		// Check for duplicate support points. This is the main termination criteria.
		var duplicate:Boolean = false;
		for (i = 0; i < saveCount; i++)
		{
			if (vertex.indexA == saveA[i] && vertex.indexB == saveB[i])
			{
				duplicate = true;
				break;
			}
		}
		
		// If we found a duplicate support point we must exist to avoid cycling
		if (duplicate)
		{
			break;
		}
		
		// New vertex is ok and needed.
		++simplex.m_count;
	}
	
	b2_gjkMaxIters = b2Math.b2Max(b2_gjkMaxIters, iter);
	
	// Prepare output
	simplex.GetWitnessPoints(output.pointA, output.pointB);
	output.distance = b2Math.SubtractVV(output.pointA, output.pointB).Length();
	output.iterations = iter;
	
	// Cache the simplex
	simplex.WriteCache(cache);
	
	// Apply radii if requested.
	if (input.useRadii)
	{
		var rA:Number = proxyA.m_radius;
		var rB:Number = proxyB.m_radius;
		
		if (output.distance > rA + rB && output.distance > Number.MIN_VALUE)
		{
			// Shapes are still not overlapped.
			// Move the witness points to the outer surface.
			output.distance -= rA + rB;
			var normal:b2Vec2 = b2Math.SubtractVV(output.pointB, output.pointA);
			normal.Normalize();
			output.pointA.x += rA * normal.x;
			output.pointA.y += rA * normal.y;
			output.pointB.x -= rB * normal.x;
			output.pointB.y -= rB * normal.y;
		}
		else
		{
			// Shapes are overlapped when radii are considered.
			// Move the witness points to the middle.
			p = new b2Vec2();
			p.x = .5 * (output.pointA.x + output.pointB.x);
			p.y = .5 * (output.pointA.y + output.pointB.y);
			output.pointA.x = output.pointB.x = p.x;
			output.pointA.y = output.pointB.y = p.y;
			output.distance = 0.0;
		}
	}
}

};


}

import Box2D.Collision.*;
import Box2D.Collision.Shapes.*;
import Box2D.Common.*;
import Box2D.Common.Math.*;

internal class b2SimplexVertex
{
	public function Set(other:b2SimplexVertex):void
	{
		wA.SetV(other.wA);
		wB.SetV(other.wB);
		w.SetV(other.w);
		a = other.a;
		indexA = other.indexA;
		indexB = other.indexB;
	}
	
	public var wA:b2Vec2;		// support point in proxyA
	public var wB:b2Vec2;		// support point in proxyB
	public var w:b2Vec2;		// wB - wA
	public var a:Number;		// barycentric coordinate for closest point
	public var indexA:int;	// wA index
	public var indexB:int;	// wB index
}

internal class b2Simplex
{

public function ReadCache(cache:b2SimplexCache, 
			proxyA:b2DistanceProxy, transformA:b2Transform,
			proxyB:b2DistanceProxy, transformB:b2Transform):void
{
	b2Settings.b2Assert(0 <= cache.count && cache.count <= 3);
	
	var wALocal:b2Vec2;
	var wBLocal:b2Vec2;
	
	// Copy data from cache.
	m_count = cache.count;
	var vertices:Array/*b2SimplexVertex*/ = m_vertices;
	for (var i:int = 0; i < m_count; i++)
	{
		var v:b2SimplexVertex = vertices[i];
		v.indexA = cache.indexA[i];
		v.indexB = cache.indexB[i];
		wALocal = proxyA.GetVertex(v.indexA);
		wBLocal = proxyB.GetVertex(v.indexB);
		v.wA = b2Math.b2MulX(transformA, wALocal);
		v.wB = b2Math.b2MulX(transformB, wALocal);
		v.w = b2Math.SubtractVV(v.wB, v.wA);
		v.a = 0;
	}
	
	// Compute the new simplex metric, if it substantially different than
	// old metric then flush the simplex
	if (m_count > 1)
	{
		var metric1:Number = cache.metric;
		var metric2:Number = GetMetric();
		if (metric2 < .5 * metric1 || 2.0 * metric1 < metric2 || metric2 < Number.MIN_VALUE)
		{
			// Reset the simplex
			m_count = 0;
		}
	}
	
	// If the cache is empty or invalid
	if (m_count == 0)
	{
		v = vertices[0];
		v.indexA = 0;
		v.indexB = 0;
		wALocal = proxyA.GetVertex(0);
		wBLocal = proxyB.GetVertex(0);
		v.wA = b2Math.b2MulX(transformA, wALocal);
		v.wB = b2Math.b2MulX(transformB, wBLocal);
		v.w = b2Math.SubtractVV(v.wB, v.wA);
		m_count = 1;
	}
}

public function WriteCache(cache:b2SimplexCache):void
{
	cache.metric = GetMetric();
	cache.count = uint(m_count);
	var vertices:Array/*b2SimplexVertex*/ = m_vertices;
	for (var i:int = 0; i < m_count; i++)
	{
		cache.indexA[i] = uint(vertices[i].indexA);
		cache.indexB[i] = uint(vertices[i].indexB);
	}
}

public function GetSearchDirection():b2Vec2
{
	switch(m_count)
	{
		case 1:
			return m_v1.w.Negative();
			
		case 2:
		{
			var e12:b2Vec2 = b2Math.SubtractVV(m_v2.w, m_v1.w);
			var sgn:Number = b2Math.b2CrossVV(e12, m_v1.w.Negative());
			if (sgn > 0.0)
			{
				// Origin is left of e12.
				return b2Math.b2CrossFV(1.0, e12);
			}else {
				// Origin is right of e12.
				return b2Math.b2CrossVF(e12, 1.0);
			}
		}
		default:
		b2Settings.b2Assert(false);
		return new b2Vec2();
	}
}

public function GetClosestPoint():b2Vec2
{
	switch(m_count)
	{
		case 0:
			b2Settings.b2Assert(false);
			return new b2Vec2();
		case 1:
			return m_v1.w;
		case 2:
			return new b2Vec2(
					m_v1.a * m_v1.w.x + m_v2.a * m_v2.w.x,
					m_v1.a * m_v1.w.y + m_v2.a * m_v2.w.y);
		default:
			b2Settings.b2Assert(false);
			return new b2Vec2();
	}
}

public function GetWitnessPoints(pA:b2Vec2, pB:b2Vec2):void
{
	switch(m_count)
	{
		case 0:
			b2Settings.b2Assert(false);
			break;
		case 1:
			pA.SetV(m_v1.wA);
			pB.SetV(m_v1.wB);
			break;
		case 2:
			pA.x = m_v1.a * m_v1.wA.x + m_v2.a * m_v2.wA.x;
			pA.y = m_v1.a * m_v1.wA.y + m_v2.a * m_v2.wA.y;
			pB.x = m_v1.a * m_v1.wB.x + m_v2.a * m_v2.wB.x;
			pB.y = m_v1.a * m_v1.wB.y + m_v2.a * m_v2.wB.y;
			break;
		case 3:
			pB.x = pA.x = m_v1.a * m_v1.wA.x + m_v2.a * m_v2.wA.x + m_v3.a * m_v3.wA.x;
			pB.y = pA.y = m_v1.a * m_v1.wA.y + m_v2.a * m_v2.wA.y + m_v3.a * m_v3.wA.y;
			break;
		default:
			b2Settings.b2Assert(false);
			break;
	}
}

public function GetMetric():Number
{
	switch (m_count)
	{
	case 0:
		b2Settings.b2Assert(false);
		return 0.0;

	case 1:
		return 0.0;

	case 2:
		return b2Math.SubtractVV(m_v1.w, m_v2.w).Length();

	case 3:
		return b2Math.b2CrossVV(b2Math.SubtractVV(m_v2.w, m_v1.w),b2Math.SubtractVV(m_v3.w, m_v1.w));

	default:
		b2Settings.b2Assert(false);
		return 0.0;
	}
}

// Solve a line segment using barycentric coordinates.
//
// p = a1 * w1 + a2 * w2
// a1 + a2 = 1
//
// The vector from the origin to the closest point on the line is
// perpendicular to the line.
// e12 = w2 - w1
// dot(p, e) = 0
// a1 * dot(w1, e) + a2 * dot(w2, e) = 0
//
// 2-by-2 linear system
// [1      1     ][a1] = [1]
// [w1.e12 w2.e12][a2] = [0]
//
// Define
// d12_1 =  dot(w2, e12)
// d12_2 = -dot(w1, e12)
// d12 = d12_1 + d12_2
//
// Solution
// a1 = d12_1 / d12
// a2 = d12_2 / d12
public function Solve2():void
{
	var w1:b2Vec2 = m_v1.w;
	var w2:b2Vec2 = m_v2.w;
	var e12:b2Vec2 = b2Math.SubtractVV(w2, w1);
	
	// w1 region
	var d12_2:Number = -(w1.x * e12.x + w1.y * e12.y);
	if (d12_2 <= 0.0)
	{
		// a2 <= 0, so we clamp it to 0
		m_v1.a = 1.0;
		m_count = 1;
		return;
	}
	
	// w2 region
	var d12_1:Number = (w2.x * e12.x + w2.y * e12.y);
	if (d12_1 <= 0.0)
	{
		// a1 <= 0, so we clamp it to 0
		m_v2.a = 1.0;
		m_count = 1;
		m_v1.Set(m_v2);
		return;
	}
	
	// Must be in e12 region.
	var inv_d12:Number = 1.0 / (d12_1 + d12_2);
	m_v1.a = d12_1 * inv_d12;
	m_v2.a = d12_2 * inv_d12;
	m_count = 2;
}

public function Solve3():void
{
	var w1:b2Vec2 = m_v1.w;
	var w2:b2Vec2 = m_v2.w;
	var w3:b2Vec2 = m_v3.w;
	
	// Edge12
	// [1      1     ][a1] = [1]
	// [w1.e12 w2.e12][a2] = [0]
	// a3 = 0
	var e12:b2Vec2 = b2Math.SubtractVV(w2, w1);
	var w1e12:Number = b2Math.b2Dot(w1, e12);
	var w2e12:Number = b2Math.b2Dot(w2, e12);
	var d12_1:Number = w2e12;
	var d12_2:Number = -w1e12;

	// Edge13
	// [1      1     ][a1] = [1]
	// [w1.e13 w3.e13][a3] = [0]
	// a2 = 0
	var e13:b2Vec2 = b2Math.SubtractVV(w3, w1);
	var w1e13:Number = b2Math.b2Dot(w1, e13);
	var w3e13:Number = b2Math.b2Dot(w3, e13);
	var d13_1:Number = w3e13;
	var d13_2:Number = -w1e13;

	// Edge23
	// [1      1     ][a2] = [1]
	// [w2.e23 w3.e23][a3] = [0]
	// a1 = 0
	var e23:b2Vec2 = b2Math.SubtractVV(w3, w2);
	var w2e23:Number = b2Math.b2Dot(w2, e23);
	var w3e23:Number = b2Math.b2Dot(w3, e23);
	var d23_1:Number = w3e23;
	var d23_2:Number = -w2e23;
	
	// Triangle123
	var n123:Number = b2Math.b2CrossVV(e12, e13);

	var d123_1:Number = n123 * b2Math.b2CrossVV(w2, w3);
	var d123_2:Number = n123 * b2Math.b2CrossVV(w3, w1);
	var d123_3:Number = n123 * b2Math.b2CrossVV(w1, w2);

	// w1 region
	if (d12_2 <= 0.0 && d13_2 <= 0.0)
	{
		m_v1.a = 1.0;
		m_count = 1;
		return;
	}

	// e12
	if (d12_1 > 0.0 && d12_2 > 0.0 && d123_3 <= 0.0)
	{
		var inv_d12:Number = 1.0 / (d12_1 + d12_2);
		m_v1.a = d12_1 * inv_d12;
		m_v2.a = d12_1 * inv_d12;
		m_count = 2;
		return;
	}

	// e13
	if (d13_1 > 0.0 && d13_2 > 0.0 && d123_2 <= 0.0)
	{
		var inv_d13:Number = 1.0 / (d13_1 + d13_2);
		m_v1.a = d13_1 * inv_d13;
		m_v3.a = d13_2 * inv_d13;
		m_count = 2;
		m_v2.Set(m_v3);
		return;
	}

	// w2 region
	if (d12_1 <= 0.0 && d23_2 <= 0.0)
	{
		m_v2.a = 1.0;
		m_count = 1;
		m_v1.Set(m_v2);
		return;
	}

	// w3 region
	if (d13_1 <= 0.0 && d23_1 <= 0.0)
	{
		m_v3.a = 1.0;
		m_count = 1;
		m_v1.Set(m_v3);
		return;
	}

	// e23
	if (d23_1 > 0.0 && d23_2 > 0.0 && d123_1 <= 0.0)
	{
		var inv_d23:Number = 1.0 / (d23_1 + d23_2);
		m_v2.a = d23_1 * inv_d23;
		m_v3.a = d23_2 * inv_d23;
		m_count = 2;
		m_v1.Set(m_v3);
		return;
	}

	// Must be in triangle123
	var inv_d123:Number = 1.0 / (d123_1 + d123_2 + d123_3);
	m_v1.a = d123_1 * inv_d123;
	m_v2.a = d123_2 * inv_d123;
	m_v3.a = d123_3 * inv_d123;
	m_count = 3;
}

public var m_v1:b2SimplexVertex = new b2SimplexVertex();
public var m_v2:b2SimplexVertex = new b2SimplexVertex();
public var m_v3:b2SimplexVertex = new b2SimplexVertex();
public var m_vertices:Array = [m_v1, m_v2, m_v3];
public var m_count:int;
}