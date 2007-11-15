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

package Engine.Collision.Shapes{



import Engine.Common.Math.*;
import Engine.Common.*
import Engine.Collision.Shapes.*;
import Engine.Dynamics.*
import Engine.Collision.*



public class b2CircleShape extends b2Shape
{
	public override function TestPoint(p:b2Vec2):Boolean{
		//var d:b2Vec2 = b2Math.SubtractVV(p, m_position);
		var d:b2Vec2 = new b2Vec2();
		d.SetV(p);
		d.Subtract(m_position);
		return b2Math.b2Dot(d, d) <= m_radius * m_radius;
	}

	//--------------- Internals Below -------------------

	public function b2CircleShape(def:b2ShapeDef, body:b2Body, localCenter:b2Vec2){
		super(def, body);
		
		//b2Settings.b2Assert(def.type == b2Shape.e_circleShape);
		var circle:b2CircleDef = def as b2CircleDef;
		
		//m_localPosition = def.localPosition - localCenter;
		m_localPosition.Set(def.localPosition.x - localCenter.x, def.localPosition.y - localCenter.y);
		m_type = b2Shape.e_circleShape;
		m_radius = circle.radius;
		
		m_R.SetM(m_body.m_R);
		//m_position = m_body.m_position + b2Mul(m_body.m_R, m_localPosition);
		m_position.SetV(m_localPosition);
		m_position.MulM(m_R);
		m_position.Add(m_body.m_position);
		
		var aabb:b2AABB = new b2AABB();
		aabb.minVertex.Set(m_position.x - m_radius, m_position.y - m_radius);
		aabb.maxVertex.Set(m_position.x + m_radius, m_position.y + m_radius);
		
		var broadPhase:b2BroadPhase = m_body.m_world.m_broadPhase;
		if (broadPhase.InRange(aabb))
		{
			m_proxyId = broadPhase.CreateProxy(aabb, def.groupIndex, def.categoryBits, def.maskBits, this);
		}
		else
		{
			m_proxyId = b2Pair.b2_nullProxy;
		}
		
		if (m_proxyId == b2Pair.b2_nullProxy)
		{
			m_body.Freeze();
		}
	}

	public override function Synchronize(position:b2Vec2, R:b2Mat22){
		
		m_R.SetM(R);
		//m_position = position + b2Mul(R, m_localPosition);
		m_position.SetV(m_localPosition);
		m_position.MulM(R);
		m_position.Add(position);
		
		if (m_proxyId == b2Pair.b2_nullProxy)
		{	
			return;
		}
		
		var aabb:b2AABB = new b2AABB();
		aabb.minVertex.Set(m_position.x - m_radius, m_position.y - m_radius);
		aabb.maxVertex.Set(m_position.x + m_radius, m_position.y + m_radius);
		
		var broadPhase:b2BroadPhase = m_body.m_world.m_broadPhase;
		if (broadPhase.InRange(aabb))
		{
			broadPhase.MoveProxy(m_proxyId, aabb);
		}
		else
		{
			broadPhase.DestroyProxy(m_proxyId);
			m_proxyId = b2Pair.b2_nullProxy;
			m_body.Freeze();
		}
	}
	
	
	public override function ResetProxy(broadPhase:b2BroadPhase)
	{
		if (m_proxyId == b2Pair.b2_nullProxy)
		{	
			return;
		}
		
		var proxy:b2Proxy = broadPhase.GetProxy(m_proxyId);
		var groupIndex:int = proxy.groupIndex;
		var categoryBits:uint = proxy.categoryBits;
		var maskBits:uint = proxy.maskBits;
		
		broadPhase.DestroyProxy(m_proxyId);
		proxy = null;
		
		var aabb:b2AABB = new b2AABB();
		aabb.minVertex.Set(m_position.x - m_radius, m_position.y - m_radius);
		aabb.maxVertex.Set(m_position.x + m_radius, m_position.y + m_radius);
		
		if (broadPhase.InRange(aabb))
		{
			m_proxyId = broadPhase.CreateProxy(aabb, groupIndex, categoryBits, maskBits, this);
		}
		else
		{
			m_proxyId = b2Pair.b2_nullProxy;
		}
		
		if (m_proxyId == b2Pair.b2_nullProxy)
		{
			m_body.Freeze();
		}
	}
	
	
	public override function Support(dX:Number, dY:Number):b2Vec2
	{
		//b2Vec2 u = d;
		//u.Normalize();
		var len:Number = Math.sqrt(dX*dX + dY*dY);
		dX /= len;
		dY /= len;
		//return m_position + m_radius * u;
		return new b2Vec2(	m_position.x + m_radius*dX, 
							m_position.y + m_radius*dY);
	}
	
	
	// Local position in parent body
	public var m_localPosition:b2Vec2 = new b2Vec2();
	public var m_radius:Number;
};

}