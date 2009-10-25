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


import Box2D.Dynamics.*;
import Box2D.Dynamics.Contacts.*;
import Box2D.Collision.Shapes.*;
import Box2D.Collision.*;
import Box2D.Common.Math.*;
import Box2D.Common.*;

import Box2D.Common.b2internal;
use namespace b2internal;


//typedef b2Contact* b2ContactCreateFcn(b2Shape* shape1, b2Shape* shape2, b2BlockAllocator* allocator);
//typedef void b2ContactDestroyFcn(b2Contact* contact, b2BlockAllocator* allocator);



/**
* The class manages contact between two shapes. A contact exists for each overlapping
* AABB in the broad-phase (except if filtered). Therefore a contact object may exist
* that has no contact points.
*/
public class b2Contact
{
	/**
	 * Get the contact manifold. Do not set the point count to zero. Instead
	 * call disable
	 */
	public function GetManifold():b2Manifold
	{
		return m_manifold;
	}
	
	/**
	 * Get the world manifold
	 */
	public function GetWorldManifold(worldManifold:b2WorldManifold):void
	{
		var bodyA:b2Body = m_fixtureA.GetBody();
		var bodyB:b2Body = m_fixtureB.GetBody();
		var shapeA:b2Shape = m_fixtureA.GetShape();
		var shapeB:b2Shape = m_fixtureB.GetShape();
		
		worldManifold.Initialize(m_manifold, bodyA.GetTransform(), shapeA.m_radius, bodyB.GetTransform(), shapeB.m_radius);
	}
	
	/**
	 * Is this contact solid? Returns fals if the shapes are separate,
	 * sensors, or the contact has been disabled.
	 * @return true if this contact should generate a response.
	 */
	public function IsSolid():Boolean
	{
		return (m_flags & (e_sensorFlag | e_disabledFlag)) == 0;
	}
	
	/**
	 * Is this contact touching.
	 */
	public function IsTouching():Boolean
	{
		return (m_flags & e_touchingFlag) != 0; 
	}
	
	/**
	 * Does this contact generate TOI events for continuous simulation
	 */
	public function IsContinuous():Boolean
	{
		return (m_flags & e_continuousFlag) != 0; 
	}
	
	/**
	 * Change this to be a sensor or-non-sensor contact.
	 */
	public function SetAsSensor(sensor:Boolean):void
	{
		if (sensor)
		{
			m_flags |= e_sensorFlag;
		}
		else
		{
			m_flags &= ~e_sensorFlag;
		}
	}
	
	/**
	 * Disable this contact. This can be used inside the pre-solve
	 * contact listener. The contact is only disabled for the current
	 * time step (or sub-step in continuous collision).
	 */
	public function Disable():void
	{
		m_flags |= e_disabledFlag;
	}
	
	/**
	* Get the next contact in the world's contact list.
	*/
	public function GetNext():b2Contact{
		return m_next;
	}
	
	/**
	* Get the first fixture in this contact.
	*/
	public function GetFixtureA():b2Fixture
	{
		return m_fixtureA;
	}
	
	/**
	* Get the second fixture in this contact.
	*/
	public function GetFixtureB():b2Fixture
	{
		return m_fixtureB;
	}
	
	/**
	 * Flag this contact for filtering. Filtering will occur the next time step.
	 */
	public function FlagForFiltering():void
	{
		m_flags |= e_filterFlag;
	}

	//--------------- Internals Below -------------------
	
	// m_flags
	// enum
	// This contact should not participate in Solve
	// The contact equivalent of sensors
	static b2internal var e_sensorFlag:uint		= 0x0001;
	// Generate TOI events.
	static b2internal var e_continuousFlag:uint	= 0x0002;
	// Used when crawling contact graph when forming islands.
	static b2internal var e_islandFlag:uint		= 0x0004;
	// Used in SolveTOI to indicate the cached toi value is still valid.
	static b2internal var e_toiFlag:uint		= 0x0008;
	// Set when shapes are touching
	static b2internal var e_touchingFlag:uint	= 0x0010;
	// Disabled (by user)
	static b2internal var e_disabledFlag:uint	= 0x0020;
	// This contact needs filtering because a fixture filter was changed.
	static b2internal var e_filterFlag:uint		= 0x0040;

	static b2internal function AddType(createFcn:Function, destroyFcn:Function, type1:int, type2:int) : void
	{
		//b2Settings.b2Assert(b2Shape.e_unknownShape < type1 && type1 < b2Shape.e_shapeTypeCount);
		//b2Settings.b2Assert(b2Shape.e_unknownShape < type2 && type2 < b2Shape.e_shapeTypeCount);
		
		s_registers[type1][type2].createFcn = createFcn;
		s_registers[type1][type2].destroyFcn = destroyFcn;
		s_registers[type1][type2].primary = true;
		
		if (type1 != type2)
		{
			s_registers[type2][type1].createFcn = createFcn;
			s_registers[type2][type1].destroyFcn = destroyFcn;
			s_registers[type2][type1].primary = false;
		}
	}
	static b2internal function InitializeRegisters() : void{
		s_registers = new Array(b2Shape.e_shapeTypeCount);
		for (var i:int = 0; i < b2Shape.e_shapeTypeCount; i++){
			s_registers[i] = new Array(b2Shape.e_shapeTypeCount);
			for (var j:int = 0; j < b2Shape.e_shapeTypeCount; j++){
				s_registers[i][j] = new b2ContactRegister();
			}
		}
		
		AddType(b2CircleContact.Create, b2CircleContact.Destroy, b2Shape.e_circleShape, b2Shape.e_circleShape);
		AddType(b2PolyAndCircleContact.Create, b2PolyAndCircleContact.Destroy, b2Shape.e_polygonShape, b2Shape.e_circleShape);
		AddType(b2PolygonContact.Create, b2PolygonContact.Destroy, b2Shape.e_polygonShape, b2Shape.e_polygonShape);
		
		AddType(b2EdgeAndCircleContact.Create, b2EdgeAndCircleContact.Destroy, b2Shape.e_edgeShape, b2Shape.e_circleShape);
		AddType(b2PolyAndEdgeContact.Create, b2PolyAndEdgeContact.Destroy, b2Shape.e_polygonShape, b2Shape.e_edgeShape);
	}
	static b2internal function Create(fixtureA:b2Fixture, fixtureB:b2Fixture, allocator:*):b2Contact{
		if (s_initialized == false)
		{
			InitializeRegisters();
			s_initialized = true;
		}
		
		var type1:int = fixtureA.GetType();
		var type2:int = fixtureB.GetType();
		
		//b2Settings.b2Assert(b2Shape.e_unknownShape < type1 && type1 < b2Shape.e_shapeTypeCount);
		//b2Settings.b2Assert(b2Shape.e_unknownShape < type2 && type2 < b2Shape.e_shapeTypeCount);
		
		var reg:b2ContactRegister = s_registers[type1][type2];
		var createFcn:Function = reg.createFcn;
		if (createFcn != null)
		{
			if (reg.primary)
			{
				return createFcn(fixtureA, fixtureB, allocator);
			}
			else
			{
				return createFcn(fixtureB, fixtureA, allocator);
				var c:b2Contact = createFcn(fixtureB, fixtureA, allocator);
				for (var i:int = 0; i < c.m_manifoldCount; ++i)
				{
					var m:b2Manifold = c.GetManifold()[ i ];
					m.normal = m.normal.Negative();
				}
				return c;
			}
		}
		else
		{
			return null;
		}
	}
	static b2internal function Destroy(contact:b2Contact, allocator:*) : void{
		//b2Settings.b2Assert(s_initialized == true);
		
		if (contact.m_manifold.m_pointCount > 0)
		{
			contact.m_fixtureA.m_body.WakeUp();
			contact.m_fixtureB.m_body.WakeUp();
		}
		
		var type1:int = contact.m_fixtureA.GetType();
		var type2:int = contact.m_fixtureB.GetType();
		
		//b2Settings.b2Assert(b2Shape.e_unknownShape < type1 && type1 < b2Shape.e_shapeTypeCount);
		//b2Settings.b2Assert(b2Shape.e_unknownShape < type2 && type2 < b2Shape.e_shapeTypeCount);
		
		var reg:b2ContactRegister = s_registers[type1][type2];
		var destroyFcn:Function = reg.destroyFcn;
		destroyFcn(contact, allocator);
	}

	/** @private */
	public function b2Contact(fixtureA:b2Fixture=null, fixtureB:b2Fixture=null)
	{
		m_flags = 0;
		
		if (!fixtureA || !fixtureB){
			m_fixtureA = null;
			m_fixtureB = null;
			return;
		}
		
		if (fixtureA.IsSensor() || fixtureB.IsSensor())
		{
			m_flags |= e_sensorFlag;
		}
		
		var bodyA:b2Body = fixtureA.GetBody();
		var bodyB:b2Body = fixtureB.GetBody();
		
		if (bodyA.IsStatic() || bodyA.IsBullet() || bodyB.IsStatic() || bodyB.IsBullet())
		{
			m_flags |= e_continuousFlag;
		}else{
			m_flags &= ~e_continuousFlag;
		}
		
		m_fixtureA = fixtureA;
		m_fixtureB = fixtureB;
		
		m_manifold.m_pointCount = 0;
		
		m_prev = null;
		m_next = null;
		
		m_nodeA.contact = null;
		m_nodeA.prev = null;
		m_nodeA.next = null;
		m_nodeA.other = null;
		
		m_nodeB.contact = null;
		m_nodeB.prev = null;
		m_nodeB.next = null;
		m_nodeB.other = null;
	}
	
	b2internal function Update(listener:b2ContactListener) : void
	{
		//TODO: Avoid a copy here
		// Perhaps an alternating scheme?
		var oldManifold:b2Manifold = m_manifold.Copy();
		
		// Re-enable this contact
		m_flags &= ~e_disabledFlag;
		
		if (m_fixtureA.m_aabb.TestOverlap(m_fixtureB.m_aabb))
		{
			Evaluate();
		}
		else
		{
			m_manifold.m_pointCount = 0;
		}
		
		var bodyA:b2Body = m_fixtureA.m_body;
		var bodyB:b2Body = m_fixtureB.m_body;
		
		var oldCount:Number = oldManifold.m_pointCount;
		var newCount:Number = m_manifold.m_pointCount;
		
		if (newCount == 0 && oldCount > 0)
		{
			bodyA.WakeUp();
			bodyB.WakeUp();
		}
		
		// Slow contacts don't generate TOI events.
		if (bodyA.IsStatic() || bodyA.IsBullet() || bodyB.IsStatic() || bodyB.IsBullet())
		{
			m_flags |= e_continuousFlag;
		}
		else
		{
			m_flags &= ~e_continuousFlag;
		}
		
		// Match old contact ids to new contact ids and copy the
		// stored impulses to warm start the solver.
		for (var i:int = 0; i < m_manifold.m_pointCount; ++i)
		{
			var mp2:b2ManifoldPoint = m_manifold.m_points[i];
			mp2.m_normalImpulse = 0.0;
			mp2.m_tangentImpulse = 0.0;
			var id2:b2ContactID = mp2.m_id;

			for (var j:int = 0; j < oldManifold.m_pointCount; ++j)
			{
				var mp1:b2ManifoldPoint = oldManifold.m_points[j];

				if (mp1.m_id.key == id2.key)
				{
					mp2.m_normalImpulse = mp1.m_normalImpulse;
					mp2.m_tangentImpulse = mp1.m_tangentImpulse;
					break;
				}
			}
		}
		
		if (newCount > 0)
		{
			m_flags |= e_touchingFlag;
		}
		else
		{
			m_flags &= ~e_touchingFlag;
		}

		if (oldCount == 0 && newCount > 0)
		{
			listener.BeginContact(this);
		}

		if (oldCount > 0 && newCount == 0)
		{
			listener.EndContact(this);
		}

		if ((m_flags & e_sensorFlag) == 0)
		{
			listener.PreSolve(this, oldManifold);
		}
	}

	//virtual ~b2Contact() {}

	b2internal virtual function Evaluate() : void{};
	
	b2internal function ComputeTOI(sweepA:b2Sweep, sweepB:b2Sweep):Number
	{
		var input:b2TOIInput = new b2TOIInput();
		input.proxyA.Set(m_fixtureA.GetShape());
		input.proxyB.Set(m_fixtureB.GetShape());
		input.sweepA = sweepA;
		input.sweepB = sweepB;
		input.tolerance = b2Settings.b2_linearSlop;
		return b2TimeOfImpact.TimeOfImpact(input);
	}
	
	static b2internal var s_registers:Array; //[][]
	static b2internal var s_initialized:Boolean = false;

	b2internal var m_flags:uint;

	// World pool and list pointers.
	b2internal var m_prev:b2Contact;
	b2internal var m_next:b2Contact;

	// Nodes for connecting bodies.
	b2internal var m_nodeA:b2ContactEdge = new b2ContactEdge();
	b2internal var m_nodeB:b2ContactEdge = new b2ContactEdge();

	b2internal var m_fixtureA:b2Fixture;
	b2internal var m_fixtureB:b2Fixture;

	b2internal var m_manifold:b2Manifold = new b2Manifold;
	
	b2internal var m_toi:Number;
};


}
