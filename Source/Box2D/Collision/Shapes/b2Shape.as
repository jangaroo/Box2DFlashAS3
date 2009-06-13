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

package Box2D.Collision.Shapes{




import Box2D.Common.Math.*;
import Box2D.Common.*;
import Box2D.Dynamics.*;
import Box2D.Collision.*;

import Box2D.Common.b2internal;
use namespace b2internal;



/**
* A shape is used for collision detection. Shapes are created in b2Body.
* You can use shape for collision detection before they are attached to the world.
* @warning you cannot reuse shapes.
*/
public class b2Shape
{
	/**
	* Get the type of this shape. You can use this to down cast to the concrete shape.
	* @return the shape type.
	*/
	public function GetType() : int{
		return m_type;
	}

	/**
	* Is this shape a sensor (non-solid)?
	* @return the true if the shape is a sensor.
	*/
	public function IsSensor() : Boolean
	{
		return m_isSensor;
	}

	/**
	* Set the if the object is a sensor. You must call b2World.Refilter to correct
	* existing contacts/non-contacts.
	* @see Box2D.Dynamics.b2World#Refilter()
	*/
	public function SetSensor(sensor:Boolean) : void
	{
		m_isSensor = sensor;
	}

	/**
	* Set the contact filtering data. You must call b2World.Refilter to correct
	* existing contacts/non-contacts.
	* @see Box2D.Dynamics.b2World#Refilter()
	*/
	public function SetFilterData(filter:b2FilterData) : void
	{
		m_filter = filter.Copy();
	}

	/**
	* Get the contact filtering data.
	*/
	public function GetFilterData() : b2FilterData
	{
		return m_filter.Copy();
	}

	/**
	* Get the parent body of this shape. This is NULL if the shape is not attached.
	* @return the parent body.
	*/
	public function GetBody() : b2Body{
		return m_body;
	}

	/**
	* Get the next shape in the parent body's shape list.
	* @return the next shape.
	*/
	public function GetNext() : b2Shape{
		return m_next;
	}

	/**
	* Get the user data that was assigned in the shape definition. Use this to
	* store your application specific data.
	*/
	public function GetUserData() : *{
		return m_userData;
	}

	/**
	* Set the user data. Use this to store your application specific data.
	*/
	public function SetUserData(data:*) : void
	{
		m_userData = data;
	}

	/**
	* Test a point for containment in this shape. This only works for convex shapes.
	* @param xf the shape world transform.
	* @param p a point in world coordinates.
	*/
	public virtual function TestPoint(xf:b2XForm, p:b2Vec2) : Boolean {return false};

	/**
	* Perform a ray cast against this shape.
	* @param xf the shape world transform.
	* @param lambda returns the hit fraction. You can use this to compute the contact point:
	* p = (1 - lambda) * segment.p1 + lambda * segment.p2.
	* 
	* lambda should be an array with one member. After calling TestSegment, you can retrieve the output value with
	* lambda[0].
	* @param normal returns the normal at the contact point. If there is no intersection, the normal
	* is not set.
	* @param segment defines the begin and end point of the ray cast.
	* @param maxLambda a number typically in the range [0,1].
	* @return b2Shape.e_hitCollide if there was an intersection, b2Shape.e_startsInsideCollide if the point is inside and b2Shape.e_missCollide otherwise.
	*/
	public virtual function  TestSegment(xf:b2XForm,
								lambda:Array, // float pointer
								normal:b2Vec2, // pointer
								segment:b2Segment,
								maxLambda:Number) : int {return e_missCollide};

	/**
	* Given a transform, compute the associated axis aligned bounding box for this shape.
	* @param aabb returns the axis aligned box.
	* @param xf the world transform of the shape.
	*/
	public virtual function  ComputeAABB(aabb:b2AABB, xf:b2XForm) : void {};

	/**
	* Given two transforms, compute the associated swept axis aligned bounding box for this shape.
	* @param aabb returns the axis aligned box.
	* @param xf1 the starting shape world transform.
	* @param xf2 the ending shape world transform.
	*/
	public virtual function  ComputeSweptAABB(	aabb:b2AABB,
									xf1:b2XForm,
									xf2:b2XForm) : void {};

	/**
	* Compute the mass properties of this shape using its dimensions and density.
	* The inertia tensor is computed about the local origin, not the centroid.
	* @param massData returns the mass data for this shape.
	*/
	public virtual function  ComputeMass(massData:b2MassData) : void { };
	
	/**
	 * Compute the volume and centroid of this shape intersected with a half plane
	 * @param normal the surface normal
	 * @param offset the surface offset along normal
	 * @param xf the shape transform
	 * @param c returns the centroid
	 * @return the total volume less than offset along normal
	 */
	public virtual function ComputeSubmergedArea(
				normal:b2Vec2,
				offset:Number,
				xf:b2XForm,
				c:b2Vec2):Number { return 0; };

	/**
	* Get the maximum radius about the parent body's center of mass.
	*/
	public function GetSweepRadius() : Number
	{
		return m_sweepRadius;
	}

	/**
	* Get the coefficient of friction.
	*/
	public function GetFriction() : Number
	{
		return m_friction;
	}
	
	/**
	 * Set the coefficient of friction.
	 */
	public function SetFriction(friction:Number) : void
	{
		m_friction = friction;
	}

	/**
	* Get the coefficient of restitution.
	*/
	public function GetRestitution() : Number
	{
		return m_restitution;
	}
	
	/**
	 * Set the coefficient of restitution.
	 */
	public function SetRestitution(restitution:Number) : void
	{
		m_restitution = restitution;
	}
	
	//--------------- Internals Below -------------------

	static b2internal function Create(def:b2ShapeDef, allocator:*) : b2Shape
	{
		switch (def.type)
		{
		case e_circleShape:
			{
				//void* mem = allocator->Allocate(sizeof(b2CircleShape));
				return new b2CircleShape(def);
			}
		
		case e_polygonShape:
			{
				//void* mem = allocator->Allocate(sizeof(b2PolygonShape));
				return new b2PolygonShape(def);
			}
		
		default:
			//b2Settings.b2Assert(false);
			return null;
		}
	}
	
	static b2internal function Destroy(shape:b2Shape, allocator:*) : void
	{
		switch (shape.m_type)
		{
		/*case e_circleShape:
			//s->~b2Shape();
			//allocator->Free(s, sizeof(b2CircleShape));
			break;
		
		case e_polygonShape:
			//s->~b2Shape();
			//allocator->Free(s, sizeof(b2PolygonShape));
			break;*/
		
		case e_edgeShape:
			var edge: b2EdgeShape = shape as b2EdgeShape;
			if (edge.m_nextEdge != null) edge.m_nextEdge.m_prevEdge = null;
			if (edge.m_prevEdge != null) edge.m_prevEdge.m_nextEdge = null;
			//s->~b2Shape();
			//allocator->Free(s, sizeof(b2EdgeShape));
			break;

		default:
			//b2Settings.b2Assert(false);
		}
	}

	/**
	 * @private
	 */
	public function b2Shape(def:b2ShapeDef){
		
		m_userData = def.userData;
		m_friction = def.friction;
		m_restitution = def.restitution;
		m_density = def.density;
		m_body = null;
		m_sweepRadius = 0.0;
		
		m_next = null;
		
		m_proxy = null;
		
		m_filter = def.filter.Copy();
		
		m_isSensor = def.isSensor;
		
	}
	
	//virtual ~b2Shape();

	//
	static private var s_proxyAABB:b2AABB = new b2AABB();
	b2internal function CreateProxy(broadPhase:b2BroadPhase, transform:b2XForm) : void{
		
		//b2Settings.b2Assert(m_proxyId == b2_nullProxy);
		
		var aabb:b2AABB = s_proxyAABB;
		ComputeAABB(aabb, transform);
		
		var inRange:Boolean = broadPhase.InRange(aabb);
		
		// You are creating a shape outside the world box.
		//b2Settings.b2Assert(inRange);
		
		if (inRange)
		{
			m_proxy = broadPhase.CreateProxy(aabb, this);
		}
		else
		{
			m_proxy = null;
		}
		
	}
	
	b2internal function DestroyProxy(broadPhase:b2BroadPhase) : void{
		
		if (m_proxy)
		{
			broadPhase.DestroyProxy(m_proxy);
			m_proxy = null;
		}
		
	}
	
	//
	static private var s_syncAABB:b2AABB = new b2AABB();
	//
	b2internal function Synchronize(broadPhase:b2BroadPhase, transform1:b2XForm, transform2:b2XForm) : Boolean{
		
		if (m_proxy == null)
		{	
			return false;
		}
		
		// Compute an AABB that covers the swept shape (may miss some rotation effect).
		var aabb:b2AABB = s_syncAABB;
		ComputeSweptAABB(aabb, transform1, transform2);
		
		if (broadPhase.InRange(aabb))
		{
			broadPhase.MoveProxy(m_proxy, aabb);
			return true;
		}
		else
		{
			return false;
		}
		
	}
	
	static private var s_resetAABB:b2AABB = new b2AABB();
	b2internal function RefilterProxy(broadPhase:b2BroadPhase, transform:b2XForm) : void{
		
		if (m_proxy == null)
		{
			return;
		}
		
		broadPhase.DestroyProxy(m_proxy);
		
		var aabb:b2AABB = s_resetAABB;
		ComputeAABB(aabb, transform);
		
		var inRange:Boolean = broadPhase.InRange(aabb);
		
		if (inRange)
		{
			m_proxy = broadPhase.CreateProxy(aabb, this);
		}
		else
		{
			m_proxy = null;
		}
		
	}

	b2internal virtual function UpdateSweepRadius(center:b2Vec2) : void{};

	b2internal var m_type:int;
	b2internal var m_next:b2Shape;
	b2internal var m_body:b2Body;

	// Sweep radius relative to the parent body's center of mass.
	b2internal var m_sweepRadius:Number;

	b2internal var m_density:Number;
	b2internal var m_friction:Number;
	b2internal var m_restitution:Number;

	private var m_proxy:b2Proxy;
	private var m_filter:b2FilterData;

	private var m_isSensor:Boolean;

	private var m_userData:*;

	
	
	
	/**
	* The various collision shape types supported by Box2D.
	*/
	//enum b2ShapeType
	//{
		static b2internal const e_unknownShape:int = 	-1;
		static b2internal const e_circleShape:int = 	0;
		static b2internal const e_polygonShape:int = 	1;
		static b2internal const e_edgeShape:int =       2;
		static b2internal const e_shapeTypeCount:int = 	3;
	//};
	
	/// Possible return values for TestSegment
		/** Return value for TestSegment indicating a hit. */
		static public const e_hitCollide:int = 1;
		/** Return value for TestSegment indicating a miss. */
		static public const e_missCollide:int = 0;
		/** Return value for TestSegment indicating that the segment starting point, p1, is already inside the shape. */
		static public const e_startsInsideCollide:int = -1;
	
	
};

	
}
