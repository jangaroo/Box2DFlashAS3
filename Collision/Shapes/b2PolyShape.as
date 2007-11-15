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



public class b2PolyShape extends b2Shape
{
	public override function TestPoint(p:b2Vec2):Boolean{
		
		//var pLocal:b2Vec2 = b2Math.b2MulTMV(m_R, b2Math.SubtractVV(p, m_position));
		var pLocal:b2Vec2 = new b2Vec2();
		pLocal.SetV(p);
		pLocal.Subtract(m_position);
		pLocal.MulTM(m_R);
		
		for (var i:int = 0; i < m_vertexCount; ++i)
		{
			//var dot:Number = b2Math.b2Dot(m_normals[i], b2Math.SubtractVV(pLocal, m_vertices[i]));
			var tVec:b2Vec2 = new b2Vec2();
			tVec.SetV(pLocal);
			tVec.Subtract(m_vertices[i]);
			
			var dot:Number = b2Math.b2Dot(m_normals[i], tVec);
			if (dot > 0.0)
			{
				return false;
			}
		}
		
		return true;
	}
	
	//--------------- Internals Below -------------------
	
	public function b2PolyShape(def:b2ShapeDef, body:b2Body, localCenter:b2Vec2){
		
		super(def, body);
		
		var i:int;
		var h:b2Vec2;
		
		var tVec:b2Vec2;
		
		var aabb:b2AABB = new b2AABB();
		
		// Vertices
		m_vertices = new Array(b2Settings.b2_maxPolyVertices);
		//for (i = 0; i < b2Settings.b2_maxPolyVertices; i++)
		//	m_vertices[i] = new b2Vec2();
		
		m_nextVert = new Array();
			
		// Normals
		m_normals = new Array(b2Settings.b2_maxPolyVertices);
		//for (i = 0; i < b2Settings.b2_maxPolyVertices; i++)
		//	m_normals[i] = new b2Vec2();
		
		//b2Settings.b2Assert(def.type == e_boxShape || def.type == e_polyShape);
		m_type = b2Shape.e_polyShape;
		
		var localR:b2Mat22 = new b2Mat22(def.localRotation);
		//var localPosition:b2Vec2 = def.localPosition - localCenter;
		var localPositionX:Number = def.localPosition.x - localCenter.x;
		var localPositionY:Number = def.localPosition.y - localCenter.y;
		
		// Get the vertices transformed into the body frame.
		if (def.type == b2Shape.e_boxShape)
		{
			var box:b2BoxDef = def as b2BoxDef;
			m_vertexCount = 4;
			h = box.extents;
			//m_vertices[0] = localPosition + b2Mul(localR, b2Vec2(h.x, h.y));
			m_vertices[0] = new b2Vec2(h.x, h.y);
			tVec = m_vertices[0];
			tVec.MulM(localR);
			tVec.x += localPositionX;
			tVec.y += localPositionY;
			//m_vertices[1] = localPosition + b2Mul(localR, b2Vec2(-h.x, h.y));
			m_vertices[1] = new b2Vec2(-h.x, h.y);
			tVec = m_vertices[1];
			tVec.MulM(localR);
			tVec.x += localPositionX;
			tVec.y += localPositionY;
			//m_vertices[2] = localPosition + b2Mul(localR, b2Vec2(-h.x, -h.y));
			m_vertices[2] = new b2Vec2(-h.x, -h.y);
			tVec = m_vertices[2];
			tVec.MulM(localR);
			tVec.x += localPositionX;
			tVec.y += localPositionY;
			//m_vertices[3] = localPosition + b2Mul(localR, b2Vec2(h.x, -h.y));
			m_vertices[3] = new b2Vec2(h.x, -h.y);
			tVec = m_vertices[3];
			tVec.MulM(localR);
			tVec.x += localPositionX;
			tVec.y += localPositionY;
		}
		else
		{
			var poly:b2PolyDef = def as b2PolyDef;
			
			m_vertexCount = poly.vertexCount;
			//b2Settings.b2Assert(3 <= m_vertexCount && m_vertexCount <= b2Settings.b2_maxPolyVertices);
			for (i = 0; i < m_vertexCount; ++i)
			{
				//m_vertices[i] = localPosition + b2Mul(localR, poly->vertices[i]);
				m_vertices[i] = poly.vertices[i].Copy();
				tVec = m_vertices[i];
				tVec.MulM(localR);
				tVec.x += localPositionX;
				tVec.y += localPositionY;
			}
			
		}
			
		// Compute bounding box. TODO_ERIN optimize OBB
		var minVertex:b2Vec2 = new b2Vec2(Number.MAX_VALUE, Number.MAX_VALUE);
		var maxVertex:b2Vec2 = new b2Vec2(-Number.MAX_VALUE, -Number.MAX_VALUE);
		for (i = 0; i < m_vertexCount; ++i)
		{
			minVertex = b2Math.b2MinV(minVertex, m_vertices[i]);
			maxVertex = b2Math.b2MaxV(maxVertex, m_vertices[i]);
		}
		
		m_localOBB.R.SetIdentity();
		//m_localOBB.center = 0.5 * (minVertex + maxVertex);
		m_localOBB.center.Set((minVertex.x + maxVertex.x) * 0.5, (minVertex.y + maxVertex.y) * 0.5);
		//m_localOBB.extents = 0.5 * (maxVertex - minVertex);
		m_localOBB.extents.Set((maxVertex.x - minVertex.x) * 0.5, (maxVertex.y - minVertex.y) * 0.5);
		
		// Compute the edge normals and next index map.
		for (i = 0; i < m_vertexCount; ++i)
		{
			m_nextVert[i] = i + 1 < m_vertexCount ? i + 1 : 0;
			
			// TODO: inline that shit
			var edge:b2Vec2 = m_vertices[m_nextVert[i]].Copy();
			edge.Subtract( m_vertices[i] );
			
			//m_normals[i] = b2Math.b2CrossVF(edge, 1.0);
			m_normals[i] = new b2Vec2();
			tVec = m_normals[i];
			tVec.SetV(edge);
			tVec.CrossVF(1.0);
			
			tVec.Normalize();
		}
		
		// Ensure the polygon in convex. TODO_ERIN compute convex hull.
		for (i = 0; i < m_vertexCount; ++i)
		{
			//b2Settings.b2Assert(b2Math.b2CrossVV(m_normals[i], m_normals[m_nextVert[i]]) > 0.0);
		}
		
		// The body transform is copied for convenience.
		m_R = m_body.m_R.Copy();
		m_position.SetV( m_body.m_position );
		
		var R:b2Mat22 = b2Math.b2MulMM(m_R, m_localOBB.R);
		var absR:b2Mat22 = b2Math.b2AbsM(R);
		h = b2Math.b2MulMV(absR, m_localOBB.extents);
		//var position:b2Vec2 = m_position + b2Mul(m_R, m_localOBB.center);
		var position:b2Vec2 = b2Math.b2MulMV(m_R, m_localOBB.center);
		position.Add(m_position);
		
		//aabb.minVertex = b2Math.SubtractVV(m_position, h);
		aabb.minVertex.SetV(position);
		aabb.minVertex.Subtract(h);
		//aabb.maxVertex = b2Math.AddVV(m_position, h);
		aabb.maxVertex.SetV(position);
		aabb.maxVertex.Add(h);
		
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
		
		// The body transform is copied for convenience.
		m_R.SetM(R);
		m_position.SetV(position);
		
		if (m_proxyId == b2Pair.b2_nullProxy)
		{	
			return;
		}
		
		var obbR:b2Mat22 = b2Math.b2MulMM(m_R, m_localOBB.R);
		var absR:b2Mat22 = b2Math.b2AbsM(obbR);
		var h:b2Vec2 = b2Math.b2MulMV(absR, m_localOBB.extents);
		//var center:b2Vec2 = m_position + b2Mul(m_R, m_localOBB.center);
		var center:b2Vec2 = b2Math.b2MulMV(m_R, m_localOBB.center);
		center.Add(m_position);
		
		var aabb:b2AABB = new b2AABB();
		//aabb.minVertex = b2Math.SubtractVV(m_position, h);
		aabb.minVertex.SetV(center);
		aabb.minVertex.Subtract(h);
		
		//aabb.maxVertex = b2Math.AddVV(m_position, h);
		aabb.maxVertex.SetV(center);
		aabb.maxVertex.Add(h);
		
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
	
	public override function ResetProxy(broadPhase:b2BroadPhase){
		
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
		
		var R:b2Mat22 = b2Math.b2MulMM(m_R, m_localOBB.R);
		var absR:b2Mat22 = b2Math.b2AbsM(R);
		var h:b2Vec2 = b2Math.b2MulMV(absR, m_localOBB.extents);
		//var position:b2Vec2 = m_position + b2Mul(m_R, m_localOBB.center);
		var position:b2Vec2 = b2Math.b2MulMV(m_R, m_localOBB.center);
		position.Add(m_position);
		
		var aabb:b2AABB = new b2AABB();
		//aabb.minVertex = position - h;
		aabb.minVertex.SetV(position);
		aabb.minVertex.Subtract(h);
		//aabb.maxVertex = position + h;
		aabb.maxVertex.SetV(position);
		aabb.maxVertex.Add(h);
		
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
		//b2Vec2 dLocal = b2MulT(m_R, d);
		var dLocalX:Number = (dX*m_R.col1.x + dY*m_R.col1.y);
		var dLocalY:Number = (dX*m_R.col2.x + dY*m_R.col2.y);
		
		var bestIndex:int = 0;
		//float32 bestValue = b2Dot(m_vertices[0], dLocal);
		var bestValue:Number = (m_vertices[0].x * dLocalX + m_vertices[0].y * dLocalY);
		for (var i:int = 1; i < m_vertexCount; ++i)
		{
			//float32 value = b2Dot(m_vertices[i], dLocal);
			var value:Number = (m_vertices[i].x * dLocalX + m_vertices[i].y * dLocalY);
			if (value > bestValue)
			{
				bestIndex = i;
				bestValue = value;
			}
		}
		
		//return m_position + b2Mul(m_R, m_vertices[bestIndex]);
		return new b2Vec2(	m_position.x + (m_R.col1.x * m_vertices[bestIndex].x + m_R.col2.x * m_vertices[bestIndex].y),
							m_position.y + (m_R.col1.y * m_vertices[bestIndex].x + m_R.col2.y * m_vertices[bestIndex].y));
		
	}
	
	

	public var m_localOBB:b2OBB = new b2OBB();
	public var m_vertices:Array;
	public var m_vertexCount:int;
	public var m_normals:Array;
	public var m_nextVert:Array;
};

}