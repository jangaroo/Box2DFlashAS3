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
	
	private static var b2_maxToiIters:int = 0;
	private static var b2_maxToiRootIters:int = 0;
	
//#if 1

// This algorithm uses conservative advancement to compute the time of
// impact (TOI) of two shapes.
// Refs: Bullet, Young Kim
//
static public var s_p1:b2Vec2 = new b2Vec2();
static public var s_p2:b2Vec2 = new b2Vec2();
static public var s_xf1:b2XForm = new b2XForm();
static public var s_xf2:b2XForm = new b2XForm();
//
static public function TimeOfImpact(	shapeA:b2Shape, sweepA:b2Sweep,
								shapeB:b2Shape, sweepB:b2Sweep) : Number
{
	var math1:Number;
	var math2:Number;
	
	var r1:Number = shapeA.m_sweepRadius;
	var r2:Number = shapeB.m_sweepRadius;

	//b2Settings.b2Assert(sweepA.t0 == sweepB.t0);
	//b2Settings.b2Assert(1.0 - sweepA.t0 > Number.MIN_VALUE);

	var t0:Number = sweepA.t0;
	//b2Vec2 v1 = sweepA.c - sweepA.c0;
	var v1X:Number = sweepA.c.x - sweepA.c0.x;
	var v1Y:Number = sweepA.c.y - sweepA.c0.y;
	//b2Vec2 v2 = sweepB.c - sweepB.c0;
	var v2X:Number = sweepB.c.x - sweepB.c0.x;
	var v2Y:Number = sweepB.c.y - sweepB.c0.y;
	var omega1:Number = sweepA.a - sweepA.a0;
	var omega2:Number = sweepB.a - sweepB.a0;

	var alpha:Number = 0.0;

	var p1:b2Vec2 = s_p1;
	var p2:b2Vec2 = s_p2;
	var k_maxIterations:int = 20;	// TODO_ERIN b2Settings
	var iter:int = 0;
	//b2Vec2 normal = b2Vec2_zero;
	var normalX:Number = 0.0;
	var normalY:Number = 0.0;
	var distance:Number = 0.0;
	var targetDistance:Number = 0.0;
	for(;;)
	{
		var t:Number = (1.0 - alpha) * t0 + alpha;
		//b2XForm xf1, xf2;
		var xf1:b2XForm = s_xf1;
		var xf2:b2XForm = s_xf2;
		sweepA.GetXForm(xf1, t);
		sweepB.GetXForm(xf2, t);
		
		// Get the distance between shapes.
		distance = b2Distance.Distance(p1, p2, shapeA, xf1, shapeB, xf2);
		
		if (iter == 0)
		{
			// Compute a reasonable target distance to give some breathing room
			// for conservative advancement.
			if (distance > 2.0 * b2Settings.b2_toiSlop)
			{
				targetDistance = 1.5 * b2Settings.b2_toiSlop;
			}
			else
			{
				//targetDistance = Math.max(0.05 * b2Settings.b2_toiSlop, distance - 0.5 * b2Settings.b2_toiSlop);
				math1 = 0.05 * b2Settings.b2_toiSlop;
				math2 = distance - 0.5 * b2Settings.b2_toiSlop;
				targetDistance = math1 > math2 ? math1 : math2;
			}
		}
		
		if (distance - targetDistance < 0.05 * b2Settings.b2_toiSlop || iter == k_maxIterations)
		{
			break;
		}
		
		//normal = p2 - p1;
		normalX = p2.x - p1.x;
		normalY = p2.y - p1.y;
		//normal.Normalize();
		var nLen:Number = Math.sqrt(normalX*normalX + normalY*normalY);
		normalX /= nLen;
		normalY /= nLen;
		
		// Compute upper bound on remaining movement.
		//float32 approachVelocityBound = b2Dot(normal, v1 - v2) + b2Abs(omega1) * r1 + b2Abs(omega2) * r2;
		var approachVelocityBound:Number = 	(normalX*(v1X - v2X) + normalY*(v1Y - v2Y))
											+ (omega1 < 0 ? -omega1 : omega1) * r1 
											+ (omega2 < 0 ? -omega2 : omega2) * r2;
		//if (Math.abs(approachVelocityBound) < Number.MIN_VALUE)
		if (approachVelocityBound == 0)
		{
			alpha = 1.0;
			break;
		}
		
		// Get the conservative time increment. Don't advance all the way.
		var dAlpha:Number = (distance - targetDistance) / approachVelocityBound;
		//float32 dt = (distance - 0.5f * b2_linearSlop) / approachVelocityBound;
		var newAlpha:Number = alpha + dAlpha;
		
		// The shapes may be moving apart or a safe distance apart.
		if (newAlpha < 0.0 || 1.0 < newAlpha)
		{
			alpha = 1.0;
			break;
		}
		
		// Ensure significant advancement.
		if (newAlpha < (1.0 + 100.0 * Number.MIN_VALUE) * alpha)
		{
			break;
		}
		
		alpha = newAlpha;
		
		++iter;
	}
	
	b2_maxToiIters = b2Math.b2Max(iter, b2_maxToiIters);

	return alpha;
}

//#else
/*
// u must be a unit vector
static public function b2GetSupport(shape:b2Shape, xf:b2XForm, u:b2Vec2):b2Vec2
{
	var type:int = shape.GetType();
	switch(type)
	{
		case b2Shape.e_circleShape:
		{
			var circle:b2CircleShape = shape as b2CircleShape;
			var localP:b2Vec2 = circle.GetLocalPosition();
			var p:b2Vec2 = b2Math.b2MulX(xf, localP);
			var radius:Number = circle.GetRadius();
			p.x += radius * u.x;
			p.y += radius * u.y;
			return p;
		}
		case b2Shape.e_polygonShape:
		{
			var polygon:b2PolygonShape = shape as b2PolygonShape;
			return polygon.Support(xf, u.x, u.y);
		}
		case b2Shape.e_edgeShape:
		{
			var edge:b2EdgeShape = shape as b2EdgeShape;
			return edge.Support(xf, u.x, u.y);
		}
		default:
			return null;
	}
}

// CCD via the secant method.
static public function TimeOfImpact(	shapeA:b2Shape, sweepA:b2Sweep,
							shapeB:b2Shape, sweepB:b2Sweep) : Number
{
	//b2Settings.b2Assert(sweepA.t0 == sweepB.t0);
	b2Settings.b2Assert(1.0 - sweepA.t0 > Number.MIN_VALUE);
	
	var alpha:Number = 0.0;
	
	const k_maxIterations:int = 1000; // TODO_ERIN b2Settings
	var iter:int = 0;
	var target:Number = 0.0;
	
	const b2_toiSlop:Number = b2Settings.b2_toiSlop;
	
	for (;; )
	{
		var xfA:b2XForm = new b2XForm();
		var xfB:b2XForm = new b2XForm();
		sweepA.GetXForm(xfA, alpha);
		sweepB.GetXForm(xfB, alpha);
		
		// Get the distance between shapes
		var pA:b2Vec2 = new b2Vec2();
		var pB:b2Vec2 = new b2Vec2();
		var distance:Number = b2Distance.Distance(pA, pB, shapeA, xfA, shapeB, xfB);
		if (distance < 0.0)
		{
			alpha = 1.0;
			break;
		}
		
		if (iter == 0)
		{
			// Compute a reasonable target distance to give some breathing room
			// for conservative advancement
			if (distance > 2.0 * b2_toiSlop)
			{
				target = 1.5 * b2_toiSlop;
			}
			else
			{
				target = b2Math.b2Max(0.05 * b2_toiSlop, distance - .5 * b2_toiSlop);
			}
		}
		
		if (Math.abs(distance - target) < 0.03 * b2_toiSlop)
		{
			if (iter == 0)
			{
				alpha = 1.0;
				break;
			}
			break;
		}
		
		var n:b2Vec2 = b2Math.SubtractVV(pB, pA);
		n.Normalize();
		
		var vA:b2Vec2 = b2GetSupport(shapeA, xfA, n);
		var vB:b2Vec2 = b2GetSupport(shapeB, xfB, n);
		var distanceCheck:Number = b2Math.b2Dot(n, b2Math.SubtractVV(vB, vA));
		
		var newAlpha:Number = alpha;
		{
			var a1:Number = alpha;
			var a2:Number = 1.0;
			var c1:Number = distance;
			
			sweepA.GetXForm(xfA, a2);
			sweepB.GetXForm(xfB, a2);
			vA = b2GetSupport(shapeA, xfA, n);
			vB = b2GetSupport(shapeB, xfB, n.Negative());
			
			var c2:Number = b2Math.b2Dot(n, b2Math.SubtractVV(vB, vA));
			
			// If intervals don't overlap at t2, then we are done
			if (c2 >= target)
			{
				newAlpha = 1.0;
				break;
			}
			
			// Determine when intervals intersect.
			var rootIterCount:int = 0;
			for (;; )
			{
				// Use a mix of secant rule and bisection
				var a:Number;
				if (rootIterCount & 3)
				{
					// Secant rule to improve convergence
					a = a1 + (target - c1) * (a2 - a1) / (c2 - c1);
				}
				else
				{
					// Bisection to guarantee progress.
					a = 0.5 & (a1 + a2);
				}
				
				sweepA.GetXForm(xfA, a);
				sweepB.GetXForm(xfB, a);
				vA = b2GetSupport(shapeA, xfA, n);
				vB = b2GetSupport(shapeB, xfB, n.Negative());
				var c:Number = b2Math.b2Dot(n, b2Math.SubtractVV(vB, vA));
				
				if (b2Math.b2Abs( c - target) < 0.025 * b2_toiSlop)
				{
					newAlpha = a;
					break;
				}
				
				// Ensure we continue to bracket the root
				if (c > target)
				{
					a1 = a;
					c1 = c;
				}
				else
				{
					a2 = a;
					c2 = c;
				}
				
				++rootIterCount;
				
				b2Settings.b2Assert(rootIterCount < 50);
			}
			b2_maxToiRootIters = b2Math.b2Max(b2_maxToiRootIters, rootIterCount);
		}
		
		// Ensure significant advancement
		if (newAlpha < (1.0 + 100.0 * Number.MIN_VALUE) * alpha)
			break;
			
		alpha = newAlpha;
		
		++iter;
		
		if (iter == k_maxIterations)
			break;
	}
	
	if (iter == 6)
	{
		iter += 0;
	}
	
	b2_maxToiIters = b2Math.b2Max(b2_maxToiIters, iter);
	
	return alpha;
}*/
//#endif


}


}