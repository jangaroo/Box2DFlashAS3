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

package Box2D.Dynamics.Contacts{


import Box2D.Collision.Shapes.*;
import Box2D.Collision.*;
import Box2D.Dynamics.*;
import Box2D.Common.*;
import Box2D.Common.Math.*;

import Box2D.Common.b2internal;
use namespace b2internal;

/**
* @private
*/
public class b2EdgeAndCircleContact extends b2Contact
{
	static public function Create(shape1:b2Shape, shape2:b2Shape, allocator:*):b2Contact{
		return new b2EdgeAndCircleContact(shape1, shape2);
	}
	static public function Destroy(contact:b2Contact, allocator:*) : void{
		//
	}

	public function b2EdgeAndCircleContact(shape1:b2Shape, shape2:b2Shape){
		super(shape1, shape2);
		
		m_manifold = m_manifolds[0];
		
		//b2Settings.b2Assert(m_shape1.m_type == b2Shape.e_circleShape);
		//b2Settings.b2Assert(m_shape2.m_type == b2Shape.e_circleShape);
		m_manifold.pointCount = 0;
		var point:b2ManifoldPoint = m_manifold.points[0];
		point.normalImpulse = 0.0;
		point.tangentImpulse = 0.0;
	}
	//~b2EdgeAndCircleContact() {}
	
	static private const s_evalCP:b2ContactPoint = new b2ContactPoint();
	b2internal override function Evaluate(listener:b2ContactListener) : void{
		var v1:b2Vec2;
		var v2:b2Vec2;
		var mp0:b2ManifoldPoint;
		
		var b1:b2Body = m_shape1.m_body;
		var b2:b2Body = m_shape2.m_body;
		
		//b2Manifold m0;
		//memcpy(&m0, &m_manifold, sizeof(b2Manifold));
		// TODO: make sure this is completely necessary
		m0.Set(m_manifold);
		
		b2CollideEdgeAndCircle(m_manifold, m_shape1 as b2EdgeShape, b1.m_xf, m_shape2 as b2CircleShape, b2.m_xf);
		
		var cp:b2ContactPoint = s_evalCP;
		cp.shape1 = m_shape1;
		cp.shape2 = m_shape2;
		cp.friction = b2Settings.b2MixFriction(m_shape1.GetFriction(), m_shape2.GetFriction());
		cp.restitution = b2Settings.b2MixRestitution(m_shape1.GetRestitution(), m_shape2.GetRestitution());
		
		if (m_manifold.pointCount > 0)
		{
			m_manifoldCount = 1;
			var mp:b2ManifoldPoint = m_manifold.points[ 0 ];
			
			if (m0.pointCount == 0)
			{
				mp.normalImpulse = 0.0;
				mp.tangentImpulse = 0.0;
	
				if (listener)
				{
					cp.position = b1.GetWorldPoint(mp.localPoint1);
					v1 = b1.GetLinearVelocityFromLocalPoint(mp.localPoint1);
					v2 = b2.GetLinearVelocityFromLocalPoint(mp.localPoint2);
					cp.velocity.Set(v2.x - v1.x, v2.y - v1.y);
					cp.normal.SetV(m_manifold.normal);
					cp.separation = mp.separation;
					cp.id.key = mp.id._key;
					listener.Add(cp);
				}
			} else
			{
				mp0 = m0.points[ 0 ];
				mp.normalImpulse = mp0.normalImpulse;
				mp.tangentImpulse = mp0.tangentImpulse;
				
				if (listener)
				{
					cp.position = b1.GetWorldPoint(mp.localPoint1);
					v1 = b1.GetLinearVelocityFromLocalPoint(mp.localPoint1);
					v2 = b2.GetLinearVelocityFromLocalPoint(mp.localPoint2);
					cp.velocity.Set(v2.x - v1.x, v2.y - v1.y);
					cp.normal.SetV(m_manifold.normal);
					cp.separation = mp.separation;
					cp.id.key = mp.id._key;
					listener.Persist(cp);
				}
			}
		}
		else
		{
			m_manifoldCount = 0;
			if (m0.pointCount > 0 && listener)
			{
				mp0 = m0.points[ 0 ];
				cp.position = b1.GetWorldPoint(mp0.localPoint1);
				v1 = b1.GetLinearVelocityFromLocalPoint(mp0.localPoint1);
				v2 = b2.GetLinearVelocityFromLocalPoint(mp0.localPoint2);
				cp.velocity.Set(v2.x - v1.x, v2.y - v1.y);
				cp.normal.SetV(m0.normal);
				cp.separation = mp0.separation;
				cp.id.key = mp0.id._key;
				listener.Remove(cp);
			}
		}
	}
	
	private function b2CollideEdgeAndCircle(manifold: b2Manifold,
	                                        edge: b2EdgeShape, 
	                                        xf1: b2XForm,
	                                        circle: b2CircleShape, 
	                                        xf2: b2XForm): void
	{
		manifold.pointCount = 0;
		var tMat: b2Mat22;
		var tVec: b2Vec2;
		var dX: Number;
		var dY: Number;
		var tX: Number;
		var tY: Number;
		var tPoint:b2ManifoldPoint;
		
		//b2Vec2 c = b2Mul(xf2, circle->GetLocalPosition());
		tMat = xf2.R;
		tVec = circle.m_localPosition;
		var cX: Number = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		var cY: Number = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		
		//b2Vec2 cLocal = b2MulT(xf1, c);
		tMat = xf1.R;
		tX = cX - xf1.position.x;
		tY = cY - xf1.position.y;
		var cLocalX: Number = (tX * tMat.col1.x + tY * tMat.col1.y );
		var cLocalY: Number = (tX * tMat.col2.x + tY * tMat.col2.y );
		
		var n: b2Vec2 = edge.m_normal;
		var v1: b2Vec2 = edge.m_v1;
		var v2: b2Vec2 = edge.m_v2;
		var radius: Number = circle.m_radius;
		var separation: Number;
		
		var dirDist: Number = (cLocalX - v1.x) * edge.m_direction.x +
		                      (cLocalY - v1.y) * edge.m_direction.y;
		
		var normalCalculated: Boolean = false;
		
		if (dirDist <= 0) {
			dX = cLocalX - v1.x;
			dY = cLocalY - v1.y;
			if (dX * edge.m_cornerDir1.x + dY * edge.m_cornerDir1.y < 0) {
				return;
			}
			dX = cX - (xf1.position.x + (tMat.col1.x * v1.x + tMat.col2.x * v1.y));
			dY = cY - (xf1.position.y + (tMat.col1.y * v1.x + tMat.col2.y * v1.y));
		} else if (dirDist >= edge.m_length) {
			dX = cLocalX - v2.x;
			dY = cLocalY - v2.y;
			if (dX * edge.m_cornerDir2.x + dY * edge.m_cornerDir2.y > 0) {
				return;
			}
			dX = cX - (xf1.position.x + (tMat.col1.x * v2.x + tMat.col2.x * v2.y));
			dY = cY - (xf1.position.y + (tMat.col1.y * v2.x + tMat.col2.y * v2.y));
		} else {
			separation = (cLocalX - v1.x) * n.x + (cLocalY - v1.y) * n.y;
			if (separation > radius || separation < -radius) {
				return;
			}
			separation -= radius;
			
			//manifold.normal = b2Mul(xf1.R, n);
			tMat = xf1.R;
			manifold.normal.x = (tMat.col1.x * n.x + tMat.col2.x * n.y);
			manifold.normal.y = (tMat.col1.y * n.x + tMat.col2.y * n.y);
			
			normalCalculated = true;
		}
		
		if (!normalCalculated) {
			var distSqr: Number = dX * dX + dY * dY;
			if (distSqr > radius * radius)
			{
				return;
			}
			
			if (distSqr < Number.MIN_VALUE)
			{
				separation = -radius;
				manifold.normal.x = (tMat.col1.x * n.x + tMat.col2.x * n.y);
				manifold.normal.y = (tMat.col1.y * n.x + tMat.col2.y * n.y);
			}
			else
			{
				distSqr = Math.sqrt(distSqr);
				dX /= distSqr;
				dY /= distSqr;
				separation = distSqr - radius;
				manifold.normal.x = dX;
				manifold.normal.y = dY;
			}
		}
		
		tPoint = manifold.points[0];
		manifold.pointCount = 1;
		tPoint.id.key = 0;
		tPoint.separation = separation;
		cX = cX - radius * manifold.normal.x;
		cY = cY - radius * manifold.normal.y;
		
		tX = cX - xf1.position.x;
		tY = cY - xf1.position.y;
		tPoint.localPoint1.x = (tX * tMat.col1.x + tY * tMat.col1.y );
		tPoint.localPoint1.y = (tX * tMat.col2.x + tY * tMat.col2.y );
		
		tMat = xf2.R;
		tX = cX - xf2.position.x;
		tY = cY - xf2.position.y;
		tPoint.localPoint2.x = (tX * tMat.col1.x + tY * tMat.col1.y );
		tPoint.localPoint2.y = (tX * tMat.col2.x + tY * tMat.col2.y );
	}
	
	public override function GetManifolds():Array
	{
		return m_manifolds;
	}

	private var m_manifolds:Array = [new b2Manifold()];
	private var m_manifold:b2Manifold;
	private var m0:b2Manifold = new b2Manifold();
};

}