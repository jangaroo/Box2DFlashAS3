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
public class b2TimeOfImpact
{
	
	private static var b2_toiCalls:int = 0;
	private static var b2_toiIters:int = 0;
	private static var b2_toiMaxIters:int = 0;
	private static var b2_toiRootIters:int = 0;
	private static var b2_toiMaxRootIters:int = 0;

	public static function TimeOfImpact(input:b2TOIInput):Number
	{
		++b2_toiCalls;
		
		var proxyA:b2DistanceProxy = input.proxyA;
		var proxyB:b2DistanceProxy = input.proxyB;
		
		var sweepA:b2Sweep = input.sweepA;
		var sweepB:b2Sweep = input.sweepB;
		
		b2Settings.b2Assert(sweepA.t0 == sweepB.t0);
		b2Settings.b2Assert(1.0 - sweepA.t0 > Number.MIN_VALUE);
		
		var radius:Number = proxyA.m_radius + proxyB.m_radius;
		var tolerance:Number = input.tolerance;
		
		var alpha:Number = 0.0;
		
		const k_maxIterations:int = 1000; //TODO_ERIN b2Settings
		var iter:int = 0;
		var target:Number = 0.0;
		
		// Prepare input for distance query.
		var cache:b2SimplexCache = new b2SimplexCache();
		cache.count = 0;
		var distanceInput:b2DistanceInput = new b2DistanceInput();
		distanceInput.useRadii = false;
		
		var xfA:b2Transform = new b2Transform();
		var xfB:b2Transform = new b2Transform();
		var fcn:b2SeparationFunction = new b2SeparationFunction();
		for (;; )
		{
			sweepA.GetTransform(xfA, alpha);
			sweepB.GetTransform(xfB, alpha);
			
			// Get the distance between shapes
			distanceInput.proxyA = proxyA;
			distanceInput.proxyB = proxyB;
			distanceInput.transformA = xfA;
			distanceInput.transformB = xfB;
			var distanceOutput:b2DistanceOutput = new b2DistanceOutput();
			b2Distance.Distance(distanceOutput, cache, distanceInput);
			
			if (distanceOutput.distance <= 0.0)
			{
				alpha = 1.0;
				break;
			}
			
			fcn.Initialize(cache, proxyA, xfA, proxyB, xfB);
			
			var separation:Number = fcn.Evaluate(xfA, xfB);
			if (separation <= 0.0)
			{
				alpha = 1.0;
				break;
			}
			
			if (iter == 0)
			{
				// Compute a reasonable target distance to give some breathing room
				// for conservative advancement. We take advantage of the shape radii
				// to create additional clearance
				if (separation > radius)
				{
					target = b2Math.b2Max(radius - tolerance, 0.75 * radius);
				}
				else
				{
					target = b2Math.b2Max(separation - tolerance, 0.02 * radius);
				}
			}
			
			if (separation - target < 0.5 * tolerance)
			{
				if (iter == 0)
				{
					alpha = 1.0;
					break;
				}
				break;
			}
			
//#if 0
			// Dump the curve seen by the root finder
			//{
				//const N:int = 100;
				//var dx:Number = 1.0 / N;
				//var xs:Array/*Number*/ = new Array(N + 1);
				//var fs:Array/*Number*/ = new Array(N + 1);
				//
				//var x:Number = 0.0;
				//for (var i:int = 0; i <= N; i++)
				//{
					//sweepA.GetTransform(xfA, x);
					//sweepB.GetTransform(xfB, x);
					//var f:Number = fcn.Evaluate(xfA, xfB) - target;
					//
					//trace(x, f);
					//xs[i] = x;
					//fx[i] = f'
					//
					//x += dx;
				//}
			//}
//#endif
			// Compute 1D root of f(x) - target = 0
			var newAlpha:Number = alpha;
			{
				var x1:Number = alpha;
				var x2:Number = 1.0;
				
				var f1:Number = separation;
				
				sweepA.GetTransform(xfA, x2);
				sweepB.GetTransform(xfB, x2);
				
				var f2:Number = fcn.Evaluate(xfA, xfB);
				
				// If intervals don't overlap at t2, then we are done
				if (f2 >= target)
				{
					alpha = 1.0;
					break;
				}
				
				// Determine when intervals intersect
				var rootIterCount:int = 0;
				for (;; )
				{
					// Use a mis of the secand rule and bisection
					var x:Number;
					if (rootIterCount & 1)
					{
						// Secant rule to improve convergence
						x = x1 + (target - f1) * (x2 - x1) / (f2 - f1);
					}
					else
					{
						// Bisection to guarantee progress
						x = 0.5 * (x1 + x2);
					}
					
					sweepA.GetTransform(xfA, x);
					sweepB.GetTransform(xfB, x);
					
					var f:Number = fcn.Evaluate(xfA, xfB);
					
					if (b2Math.b2Abs(f - target) < 0.025 * tolerance)
					{
						newAlpha = x;
						break;
					}
					
					// Ensure we continue to bracket the root
					if (f > target)
					{
						x1 = x;
						f1 = f;
					}
					else
					{
						x2 = x;
						f2 = f;
					}
					
					++rootIterCount;
					++b2_toiRootIters;
					if (rootIterCount == 50)
					{
						break;
					}
				}
				
				b2_toiMaxRootIters = b2Math.b2Max(b2_toiMaxRootIters, rootIterCount);
			}
			
			// Ensure significant advancement
			if (newAlpha < (1.0 + 100.0 * Number.MIN_VALUE) * alpha)
			{
				break;
			}
			
			alpha = newAlpha;
			
			iter++;
			++b2_toiIters;
			
			if (iter == k_maxIterations)
			{
				break;
			}
		}
		
		b2_toiMaxIters = b2Math.b2Max(b2_toiMaxIters, iter);

		return alpha;
	}

}

}

import Box2D.Collision.*;
import Box2D.Collision.Shapes.*;
import Box2D.Common.*;
import Box2D.Common.Math.*;

/**
* @private
*/
internal class b2SeparationFunction
{
	//enum Type
	public static const e_points:int = 0x01;
	public static const e_faceA:int = 0x02;
	public static const e_faceB:int = 0x04;
	
	public function Initialize(cache:b2SimplexCache,
								proxyA:b2DistanceProxy, transformA:b2Transform,
								proxyB:b2DistanceProxy, transformB:b2Transform):void
	{
		m_proxyA = proxyA;
		m_proxyB = proxyB;
		var count:int = cache.count;
		b2Settings.b2Assert(0 < count && count < 3);
		
		var localPointA:b2Vec2;
		var localPointA1:b2Vec2;
		var localPointA2:b2Vec2;
		var localPointB:b2Vec2;
		var localPointB1:b2Vec2;
		var localPointB2:b2Vec2;
		var pointA:b2Vec2;
		var pointB:b2Vec2;
		var normal:b2Vec2;
		var s:Number;
		var sgn:Number;
		
		if (count == 1)
		{
			m_type = e_points;
			localPointA = m_proxyA.GetVertex(cache.indexA[0]);
			localPointB = m_proxyB.GetVertex(cache.indexB[0]);
			pointA = b2Math.b2MulX(transformA, localPointA);
			pointB = b2Math.b2MulX(transformB, localPointB);
			m_axis = b2Math.SubtractVV(pointB, pointA);
			m_axis.Normalize();
		}
		else if (cache.indexB[0] == cache.indexB[1])
		{
			// Two points on A and one on B
			m_type = e_faceA;
			localPointA1 = m_proxyA.GetVertex(cache.indexA[0]);
			localPointA2 = m_proxyA.GetVertex(cache.indexA[1]);
			localPointB = m_proxyB.GetVertex(cache.indexB[0]);
			m_localPoint.x = 0.5 * (localPointA1.x + localPointA2.x);
			m_localPoint.y = 0.5 * (localPointA1.y + localPointA2.y);
			m_axis = b2Math.b2CrossVF(b2Math.SubtractVV(localPointA2, localPointA1), 1.0);
			m_axis.Normalize();
			
			normal = b2Math.b2MulMV(transformA.R, m_axis);
			pointA = b2Math.b2MulX(transformA, m_localPoint);
			pointB = b2Math.b2MulX(transformB, localPointB);
			
			//float32 s = b2Dot(pointB - pointA, normal);
			s = (pointB.x - pointA.x) * normal.x + (pointB.y - pointA.y) * normal.y;
			if (s < 0.0)
			{
				m_axis = m_axis.Negative();
			}
		}
		else if (cache.indexA[0] == cache.indexA[0])
		{
			// Two points on B and one on A
			m_type = e_faceB;
			localPointB1 = m_proxyB.GetVertex(cache.indexB[0]);
			localPointB2 = m_proxyB.GetVertex(cache.indexB[1]);
			localPointA = m_proxyA.GetVertex(cache.indexA[0]);
			m_localPoint.x = 0.5 * (localPointB1.x + localPointB2.x);
			m_localPoint.y = 0.5 * (localPointB1.y + localPointB2.y);
			m_axis = b2Math.b2CrossVF(b2Math.SubtractVV(localPointB2, localPointB1), 1.0);
			m_axis.Normalize();
			
			normal = b2Math.b2MulMV(transformB.R, m_axis);
			pointB = b2Math.b2MulX(transformB, m_localPoint);
			pointA = b2Math.b2MulX(transformA, localPointA);
			
			//float32 s = b2Dot(pointA - pointB, normal);
			s = (pointA.x - pointB.x) * normal.x + (pointA.y - pointB.y) * normal.y;
			if (s < 0.0)
			{
				m_axis = m_axis.Negative();
			}
		}
		else
		{
			// Two points on B and two points on A.
			// The faces are parallel.
			localPointA1 = m_proxyA.GetVertex(cache.indexA[0]);
			localPointA2 = m_proxyA.GetVertex(cache.indexA[1]);
			localPointB1 = m_proxyB.GetVertex(cache.indexB[0]);
			localPointB2 = m_proxyB.GetVertex(cache.indexB[1]);
			
			var pA:b2Vec2 = b2Math.b2MulX(transformA, localPointA);
			var dA:b2Vec2 = b2Math.b2MulMV(transformA.R, b2Math.SubtractVV(localPointA2, localPointA1));
			var pB:b2Vec2 = b2Math.b2MulX(transformB, localPointB);
			var dB:b2Vec2 = b2Math.b2MulMV(transformB.R, b2Math.SubtractVV(localPointB2, localPointB1));
			
			var a:Number = dA.x * dA.x + dA.y * dA.y;
			var e:Number = dB.x * dB.x + dB.y * dB.y;
			var r:b2Vec2 = b2Math.SubtractVV(dB, dA);
			var c:Number = dA.x * r.x + dA.y * r.y;
			var f:Number = dB.x * r.x + dB.y * r.y;
			
			var b:Number = dA.x * dB.x + dA.y * dB.y;
			var denom:Number = a * e-b * b;
			
			s = 0.0;
			if (denom != 0.0)
			{
				s = b2Math.b2Clamp((b * f - c * e) / denom, 0.0, 1.0);
			}
			
			var t:Number = (b * s + f) / e;
			if (t < 0.0)
			{
				t = 0.0;
				s = b2Math.b2Clamp((b - c) / a, 0.0, 1.0);
			}
			
			//b2Vec2 localPointA = localPointA1 + s * (localPointA2 - localPointA1);
			localPointA = new b2Vec2();
			localPointA.x = localPointA1.x + s * (localPointA2.x - localPointA1.x);
			localPointA.y = localPointA1.y + s * (localPointA2.y - localPointA1.y);
			//b2Vec2 localPointB = localPointB1 + s * (localPointB2 - localPointB1);
			localPointB = new b2Vec2();
			localPointB.x = localPointB1.x + s * (localPointB2.x - localPointB1.x);
			localPointB.y = localPointB1.y + s * (localPointB2.y - localPointB1.y);
			
			if (s == 0.0 || s == 1.0)
			{
				m_type = e_faceB;
				m_axis = b2Math.b2CrossVF(b2Math.SubtractVV(localPointB2, localPointB1), 1.0);
				
				m_localPoint = localPointB;
				
				normal = b2Math.b2MulMV(transformB.R, m_axis);
				pointB = b2Math.b2MulX(transformB, m_localPoint);
				pointA = b2Math.b2MulX(transformA, localPointA);
				
				//float32 sgn = b2Dot(pointA - pointB, normal);
				sgn = (pointA.x - pointB.x) * normal.x + (pointA.y - pointB.y) * normal.y;
				if (s < 0.0)
				{
					m_axis = m_axis.Negative();
				}
			}
			else
			{
				m_type = e_faceA;
				m_axis = b2Math.b2CrossVF(b2Math.SubtractVV(localPointA2, localPointA1), 1.0);
				
				m_localPoint = localPointA;
				
				normal = b2Math.b2MulMV(transformA.R, m_axis);
				pointA = b2Math.b2MulX(transformA, m_localPoint);
				pointB = b2Math.b2MulX(transformB, localPointB);
				
				//float32 sgn = b2Dot(pointB - pointA, normal);
				sgn = (pointB.x - pointA.x) * normal.x + (pointB.y - pointA.y) * normal.y;
				if (s < 0.0)
				{
					m_axis = m_axis.Negative();
				}
			}
		}
	}
	
	public function Evaluate(transformA:b2Transform, transformB:b2Transform):Number
	{
		var axisA:b2Vec2;
		var axisB:b2Vec2;
		var localPointA:b2Vec2
		var localPointB:b2Vec2;
		var pointA:b2Vec2;
		var pointB:b2Vec2;
		var seperation:Number;
		var normal:b2Vec2;
		switch(m_type)
		{
			case e_points:
			{
				axisA = b2Math.b2MulTMV(transformA.R, m_axis);
				axisB = b2Math.b2MulTMV(transformB.R, m_axis.Negative());
				localPointA = m_proxyA.GetSupportVertex(axisA);
				localPointB = m_proxyB.GetSupportVertex(axisB);
				pointA = b2Math.b2MulX(transformA, localPointA);
				pointB = b2Math.b2MulX(transformB, localPointB);
				//float32 separation = b2Dot(pointB - pointA, m_axis);
				seperation = (pointB.x - pointA.x) * m_axis.x + (pointB.y - pointA.y) * m_axis.y;
				return seperation;
			}
			case e_faceA:
			{
				normal = b2Math.b2MulMV(transformA.R, m_axis);
				pointA = b2Math.b2MulX(transformA, m_localPoint);
				
				axisB = b2Math.b2MulTMV(transformB.R, normal.Negative());
				
				localPointB = m_proxyB.GetSupportVertex(axisB);
				pointB = b2Math.b2MulX(transformB, localPointB);
				
				//float32 separation = b2Dot(pointB - pointA, m_axis);
				seperation = (pointB.x - pointA.x) * m_axis.x + (pointB.y - pointA.y) * m_axis.y;
				return seperation;
			}
			case e_faceB:
			{
				normal = b2Math.b2MulMV(transformB.R, m_axis);
				pointB = b2Math.b2MulX(transformB, m_localPoint);
				
				axisA = b2Math.b2MulTMV(transformA.R, normal.Negative());
				
				localPointA = m_proxyA.GetSupportVertex(axisA);
				pointA = b2Math.b2MulX(transformA, localPointA);
				
				//float32 separation = b2Dot(pointA - pointB, m_axis);
				seperation = (pointA.x - pointB.x) * m_axis.x + (pointA.y - pointB.y) * m_axis.y;
				return seperation;
			}
			default:
			b2Settings.b2Assert(false);
			return 0.0;
		}
	}
	
	public var m_proxyA:b2DistanceProxy;
	public var m_proxyB:b2DistanceProxy;
	public var m_type:int;
	public var m_localPoint:b2Vec2 = new b2Vec2();
	public var m_axis:b2Vec2 = new b2Vec2();
}