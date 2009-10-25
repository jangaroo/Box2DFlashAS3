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

package Box2D.Collision 
{
import Box2D.Common.Math.*;
import Box2D.Common.*;

import Box2D.Common.b2internal;
use namespace b2internal;

/**
 * This is used to compute the current state of a contact manifold.
 */
public class b2WorldManifold 
{
	public function b2WorldManifold()
	{
		m_points = new Array(b2Settings.b2_maxManifoldPoints)
		for (var i:int = 0; i < b2Settings.b2_maxManifoldPoints; i++)
		{
			m_points[i] = new b2Vec2();
		}
	}
	/**
	 * Evaluate the manifold with supplied transforms. This assumes
	 * modest motion from the original state. This does not change the
	 * point count, impulses, etc. The radii must come from the shapes
	 * that generated the manifold.
	 */
	public function Initialize(manifold:b2Manifold,
					xfA:b2Transform, radiusA:Number,
					xfB:b2Transform, radiusB:Number):void
	{
		if (manifold.m_pointCount == 0)
		{
			return;
		}
		
		var normal:b2Vec2;
		var planePoint:b2Vec2;
		var i:int;
		var clipPoint:b2Vec2;
		
		switch(manifold.m_type)
		{
			case b2Manifold.e_circles:
			{
				var pointA:b2Vec2 = b2Math.b2MulX(xfA, manifold.m_localPoint);
				var pointB:b2Vec2 = b2Math.b2MulX(xfB, manifold.m_points[0].m_localPoint);
				normal = new b2Vec2(1, 0);
				if (b2Math.b2DistanceSquared(pointA, pointB) > Number.MIN_VALUE * Number.MIN_VALUE)
				{
					normal = b2Math.SubtractVV(pointB, pointA);
					normal.Normalize();
				}
				
				m_normal = normal;
				var cA:b2Vec2 = new b2Vec2();
				cA.x = pointA.x + radiusA * normal.x;
				cA.y = pointA.y + radiusA * normal.y;
				var cB:b2Vec2 = new b2Vec2();
				cB.x = pointB.x - radiusB * normal.x;
				cB.y = pointB.y - radiusB * normal.y;
				m_points[0].x = 0.5 * (cA.x + cB.x);
				m_points[0].y = 0.5 * (cA.y + cB.y);
			}
			break;
			case b2Manifold.e_faceA:
			{
				normal = b2Math.b2MulMV(xfA.R, manifold.m_localPlaneNormal);
				planePoint = b2Math.b2MulX(xfA, manifold.m_localPoint);
				
				// Ensure normal points from A to B
				m_normal = normal;
				for (i = 0; i < manifold.m_pointCount; i++)
				{
					clipPoint = b2Math.b2MulX(xfB, manifold.m_points[i].m_localPoint);
					//b2Vec2 cA = clipPoint + (radiusA - b2Dot(clipPoint - planePoint, normal)) * normal;
					//b2Vec2 cB = clipPoint - radiusB * normal;
					//m_points[i] = 0.5f * (cA + cB);
					m_points[i].x = clipPoint.x + 0.5 * (radiusA - (clipPoint.x - planePoint.x) * normal.x - (clipPoint.y - planePoint.y) * normal.y - radiusB ) * normal.x;
					m_points[i].y = clipPoint.y + 0.5 * (radiusA - (clipPoint.x - planePoint.x) * normal.x - (clipPoint.y - planePoint.y) * normal.y - radiusB ) * normal.y;
					
				}
			}
			break;
			case b2Manifold.e_faceB:
			{
				normal = b2Math.b2MulMV(xfB.R, manifold.m_localPlaneNormal);
				planePoint = b2Math.b2MulX(xfB, manifold.m_localPoint);
				
				// Ensure normal points from A to B
				m_normal = normal.Negative();
				for (i = 0; i < manifold.m_pointCount; i++)
				{
					clipPoint = b2Math.b2MulX(xfA, manifold.m_points[i].m_localPoint);
					//b2Vec2 cA = clipPoint - radiusA * normal;
					//b2Vec2 cB = clipPoint + (radiusB - b2Dot(clipPoint - planePoint, normal)) * normal;
					//m_points[i] = 0.5f * (cA + cB);
					m_points[i].x = clipPoint.x + 0.5 * (radiusB - (clipPoint.x - planePoint.x) * normal.x - (clipPoint.y - planePoint.y) * normal.y - radiusA ) * normal.x;
					m_points[i].y = clipPoint.y + 0.5 * (radiusB - (clipPoint.x - planePoint.x) * normal.x - (clipPoint.y - planePoint.y) * normal.y - radiusA ) * normal.y;
					
				}
			}
			break;
		}
	}

	/**
	 * world vector pointing from A to B
	 */
	public var m_normal:b2Vec2 = new b2Vec2();						
	/**
	 * world contact point (point of intersection)
	 */
	public var m_points:Array/*b2Vec2*/;
	
}
	
}