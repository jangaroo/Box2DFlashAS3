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

package Box2D.Dynamics{

import Box2D.Common.Math.*;
import Box2D.Common.*;
import Box2D.Collision.*;
import Box2D.Collision.Shapes.*;
import Box2D.Dynamics.*;
import Box2D.Dynamics.Contacts.*;
import Box2D.Dynamics.Joints.*;

import Box2D.Common.b2internal;
use namespace b2internal;


/**
 * A fixture is used to attach a shape to a body for collision detection. A fixture
 * inherits its transform from its parent. Fixtures hold additional non-geometric data
 * such as friction, collision filters, etc.
 * Fixtures are created via b2Body::CreateFixture.
 * @warning you cannot reuse fixtures.
 */
public class b2Fixture
{
	/**
	 * Get the type of the child shape. You can use this to down cast to the concrete shape.
	 * @return the shape type.
	 */
	public function GetType():int
	{
		return m_shape.GetType();
	}
	
	/**
	 * Get the child shape. You can modify the child shape, however you should not change the
	 * number of vertices because this will crash some collision caching mechanisms.
	 */
	public function GetShape():b2Shape
	{
		return m_shape;
	}
	
	/**
	 * Is this fixture a sensor (non-solid)?
	 * @return the true if the shape is a sensor.
	 */
	public function IsSensor():Boolean
	{
		return m_isSensor;
	}
	
	/**
	 * Set if this fixture is a sensor.
	 */
	public function SetSensor(sensor:Boolean):void
	{
		if ( m_isSensor == sensor)
			return;
			
		m_isSensor = sensor;
		
		if (m_body == null)
			return;
			
		// Flag associated contacts for filtering
		var edge:b2ContactEdge = m_body.GetContactList();
		while (edge)
		{
			var contact:b2Contact = edge.contact;
			var fixtureA:b2Fixture = contact.GetFixtureA();
			var fixtureB:b2Fixture = contact.GetFixtureB();
			if (fixtureA == this || fixtureB == this)
				contact.SetAsSensor(m_isSensor);
			edge = edge.next;
		}
		
	}
	
	/**
	 * Set the contact filtering data. This is an expensive operation and should
	 * not be called frequently. This will not update contacts until the next time
	 * step when either parent body is awake.
	 */
	public function SetFilterData(filter:b2FilterData):void
	{
		m_filter = filter.Copy();
		
		if (m_body)
			return;
			
		var edge:b2ContactEdge = m_body.GetContactList();
		while (edge)
		{
			var contact:b2Contact = edge.contact;
			var fixtureA:b2Fixture = contact.GetFixtureA();
			var fixtureB:b2Fixture = contact.GetFixtureB();
			if (fixtureA == this || fixtureB == this)
				contact.FlagForFiltering();
			edge = edge.next;
		}
	}
	
	/**
	 * Get the contact filtering data.
	 */
	public function GetFilterData(): b2FilterData
	{
		return m_filter.Copy();
	}
	
	/**
	 * Get the parent body of this fixture. This is NULL if the fixture is not attached.
	 * @return the parent body.
	 */
	public function GetBody():b2Body
	{
		return m_body;
	}
	
	/**
	 * Get the next fixture in the parent body's fixture list.
	 * @return the next shape.
	 */
	public function GetNext():b2Fixture
	{
		return m_next;
	}
	
	/**
	 * Get the user data that was assigned in the fixture definition. Use this to
	 * store your application specific data.
	 */
	public function GetUserData():*
	{
		return m_userData;
	}
	
	/**
	 * Set the user data. Use this to store your application specific data.
	 */
	public function SetUserData(data:*):void
	{
		m_userData = data;
	}
	
	/**
	 * Test a point for containment in this fixture. This only works for convex shapes.
	 * @param xf the shape world transform.
	 * @param p a point in world coordinates.
	 */
	public function TestPoint(p:b2Vec2):Boolean
	{
		return m_shape.TestPoint(m_body.GetTransform(), p);
	}
	
	/**
	 * Perform a ray cast against this shape.
	 * @param output the ray-cast results.
	 * @param input the ray-cast input parameters.
	 */
	public function RayCast(output:b2RayCastOutput, input:b2RayCastInput):void
	{
		return m_shape.RayCast(output, input, m_body.GetTransform());
	}
	
	/**
	 * Get the mass data for this fixture. The mass data is based on the density and
	 * the shape. The rotational inertia is about the shape's origin.
	 */
	public function GetMassData():b2MassData
	{
		return m_massData;
	}
	
	/**
	 * Get the coefficient of friction.
	 */
	public function GetFriction():Number
	{
		return m_friction;
	}
	
	/**
	 * Set the coefficient of friction.
	 */
	public function SetFriction(friction:Number):void
	{
		m_friction = friction;
	}
	
	/**
	 * Get the coefficient of restitution.
	 */
	public function GetRestitution():Number
	{
		return m_restitution;
	}
	
	/**
	 * Get the coefficient of restitution.
	 */
	public function SetRestitution(restitution:Number):void
	{
		m_restitution = restitution;
	}
	
	/**
	 * @private
	 */
	public function b2Fixture()
	{
		m_aabb = new b2AABB();
		m_userData = null;
		m_body = null;
		m_next = null;
		//m_proxyId = b2BroadPhase.e_nullProxy;
		m_shape = null;
		
		m_massData = new b2MassData();
		
		m_friction = 0.0;
		m_restitution = 0.0;
	}
	// We need separation create/destroy functions from the constructor/destructor because
	// the destructor cannot access the allocator or broad-phase (no destructor arguments allowed by C++).
	b2internal function Create(broadPhase:IBroadPhase, body:b2Body, xf:b2Transform, def:b2FixtureDef):void
	{
		m_userData = def.userData;
		m_friction = def.friction;
		
		m_body = body;
		m_next = null;
		
		m_filter = def.filter.Copy();
		
		m_isSensor = def.isSensor;
		
		m_shape = def.shape.Copy();
		
		m_shape.ComputeMass(m_massData, def.density);
		
		// Create proxy in the broad-phase
		m_shape.ComputeAABB(m_aabb, xf);
		
		m_proxy = broadPhase.CreateProxy(m_aabb, this);
	}
	
	b2internal function Destroy(broadPhase:IBroadPhase):void
	{
		// Remove proxy from the broadphase
		broadPhase.DestroyProxy(m_proxy);
		m_proxy = null;
		
		// Free the child shape
		m_shape = null;
	}
	
	b2internal function Synchronize(broadPhase:IBroadPhase, transform1:b2Transform, transform2:b2Transform):void
	{
		if (!m_proxy)
			return;
			
		// Compute an AABB that ocvers the swept shape (may miss some rotation effect)
		var aabb1:b2AABB = new b2AABB();
		var aabb2:b2AABB = new b2AABB();
		m_shape.ComputeAABB(aabb1, transform1);
		m_shape.ComputeAABB(aabb2, transform2);
		
		m_aabb.Combine(aabb1, aabb2);
		var displacement:b2Vec2 = b2Math.SubtractVV(transform2.position, transform1.position);
		broadPhase.MoveProxy(m_proxy, m_aabb, displacement);
	}
	
	b2internal var m_aabb:b2AABB;
	b2internal var m_massData:b2MassData;
	b2internal var m_next:b2Fixture;
	b2internal var m_body:b2Body;
	b2internal var m_shape:b2Shape;
	
	b2internal var m_friction:Number;
	b2internal var m_restitution:Number;
	
	b2internal var m_proxy:*;
	b2internal var m_filter:b2FilterData = new b2FilterData();
	
	b2internal var m_isSensor:Boolean;
	
	b2internal var m_userData:*;
};



}
