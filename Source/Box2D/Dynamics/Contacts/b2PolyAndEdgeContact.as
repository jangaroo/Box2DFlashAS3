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
public class b2PolyAndEdgeContact extends b2Contact{
	
	static public function Create(shape1:b2Shape, shape2:b2Shape, allocator:*):b2Contact{
		return new b2PolyAndEdgeContact(shape1, shape2);
	}
	static public function Destroy(contact:b2Contact, allocator:*): void{
	}

	public function b2PolyAndEdgeContact(shape1:b2Shape, shape2:b2Shape){
		super(shape1, shape2);
		
		m_manifold = m_manifolds[0];
		
		b2Settings.b2Assert(m_shape1.m_type == b2Shape.e_polygonShape);
		b2Settings.b2Assert(m_shape2.m_type == b2Shape.e_edgeShape);
		m_manifold.pointCount = 0;
	}
	//~b2PolyAndEdgeContact() {}

	//
	static private const s_evalCP:b2ContactPoint = new b2ContactPoint();
	//
	b2internal override function Evaluate(listener:b2ContactListener): void{
		var i:int;
		var v1:b2Vec2;
		var v2:b2Vec2;
		var mp0:b2ManifoldPoint;
		
		var b1:b2Body = m_shape1.m_body;
		var b2:b2Body = m_shape2.m_body;
		
		//b2Manifold m0;
		//memcpy(&m0, &m_manifold, sizeof(b2Manifold));
		// TODO: make sure this is completely necessary
		m0.Set(m_manifold);
		
		b2CollidePolyAndEdge(m_manifold, m_shape1 as b2PolygonShape, b1.m_xf, m_shape2 as b2EdgeShape, b2.m_xf);
		
		var persisted:Array = [false, false];
		
		var cp:b2ContactPoint = s_evalCP;
		cp.shape1 = m_shape1;
		cp.shape2 = m_shape2;
		cp.friction = b2Settings.b2MixFriction(m_shape1.GetFriction(), m_shape2.GetFriction());
		cp.restitution = b2Settings.b2MixRestitution(m_shape1.GetRestitution(), m_shape2.GetRestitution());
		
		// Match contact ids to facilitate warm starting.
		if (m_manifold.pointCount > 0)
		{
			// Match old contact ids to new contact ids and copy the
			// stored impulses to warm start the solver.
			for (i = 0; i < m_manifold.pointCount; ++i)
			{
				var mp:b2ManifoldPoint = m_manifold.points[ i ];
				mp.normalImpulse = 0.0;
				mp.tangentImpulse = 0.0;
				var found:Boolean = false;
				var idKey:uint = mp.id._key;
	
				for (var j:int = 0; j < m0.pointCount; ++j)
				{
					if (persisted[j] == true)
					{
						continue;
					}
	
					mp0 = m0.points[ j ];
	
					if (mp0.id._key == idKey)
					{
						persisted[j] = true;
						mp.normalImpulse = mp0.normalImpulse;
						mp.tangentImpulse = mp0.tangentImpulse;
	
						// A persistent point.
						found = true;
	
						// Report persistent point.
						if (listener != null)
						{
							cp.position = b1.GetWorldPoint(mp.localPoint1);
							v1 = b1.GetLinearVelocityFromLocalPoint(mp.localPoint1);
							v2 = b2.GetLinearVelocityFromLocalPoint(mp.localPoint2);
							cp.velocity.Set(v2.x - v1.x, v2.y - v1.y);
							cp.normal.SetV(m_manifold.normal);
							cp.separation = mp.separation;
							cp.id.key = idKey;
							listener.Persist(cp);
						}
						break;
					}
				}
	
				// Report added point.
				if (found == false && listener != null)
				{
					cp.position = b1.GetWorldPoint(mp.localPoint1);
					v1 = b1.GetLinearVelocityFromLocalPoint(mp.localPoint1);
					v2 = b2.GetLinearVelocityFromLocalPoint(mp.localPoint2);
					cp.velocity.Set(v2.x - v1.x, v2.y - v1.y);
					cp.normal.SetV(m_manifold.normal);
					cp.separation = mp.separation;
					cp.id.key = idKey;
					listener.Add(cp);
				}
			}
	
			m_manifoldCount = 1;
		}
		else
		{
			m_manifoldCount = 0;
		}
		
		if (listener == null)
		{
			return;
		}
		
		// Report removed points.
		for (i = 0; i < m0.pointCount; ++i)
		{
			if (persisted[i])
			{
				continue;
			}
			
			mp0 = m0.points[ i ];
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
	
	private function b2CollidePolyAndEdge(manifold: b2Manifold,
	                                                polygon: b2PolygonShape, 
	                                                xf1: b2XForm,
	                                                edge: b2EdgeShape, 
	                                                xf2: b2XForm): void
	{
		manifold.pointCount = 0;
		var tMat: b2Mat22;
		var tVec1: b2Vec2;
		var tVec2: b2Vec2;
		var tX: Number;
		var tY: Number;
		var tPoint:b2ManifoldPoint;
		var ratio: Number;
		
		//b2Vec2 v1 = b2Mul(xf2, edge->GetVertex1());
		tMat = xf2.R;
		tVec1 = edge.m_v1;
		var v1X: Number = xf2.position.x + (tMat.col1.x * tVec1.x + tMat.col2.x * tVec1.y);
		var v1Y: Number = xf2.position.y + (tMat.col1.y * tVec1.x + tMat.col2.y * tVec1.y);
		
		//b2Vec2 v2 = b2Mul(xf2, edge->GetVertex2());
		tVec1 = edge.m_v2;
		var v2X: Number = xf2.position.x + (tMat.col1.x * tVec1.x + tMat.col2.x * tVec1.y);
		var v2Y: Number = xf2.position.y + (tMat.col1.y * tVec1.x + tMat.col2.y * tVec1.y);
		
		//b2Vec2 n = b2Mul(xf2.R, edge->GetNormalVector());
		tVec1 = edge.m_normal;
		var nX: Number = (tMat.col1.x * tVec1.x + tMat.col2.x * tVec1.y);
		var nY: Number = (tMat.col1.y * tVec1.x + tMat.col2.y * tVec1.y);
		
		//b2Vec2 v1Local = b2MulT(xf1, v1);
		tMat = xf1.R;
		tX = v1X - xf1.position.x;
		tY = v1Y - xf1.position.y;
		var v1LocalX: Number = (tX * tMat.col1.x + tY * tMat.col1.y );
		var v1LocalY: Number = (tX * tMat.col2.x + tY * tMat.col2.y );
		
		//b2Vec2 v2Local = b2MulT(xf1, v2);
		tX = v2X - xf1.position.x;
		tY = v2Y - xf1.position.y;
		var v2LocalX: Number = (tX * tMat.col1.x + tY * tMat.col1.y );
		var v2LocalY: Number = (tX * tMat.col2.x + tY * tMat.col2.y );
		
		//b2Vec2 nLocal = b2MulT(xf1.R, n);
		var nLocalX: Number = (nX * tMat.col1.x + nY * tMat.col1.y );
		var nLocalY: Number = (nX * tMat.col2.x + nY * tMat.col2.y );
		
		var separation1: Number;
		var separationIndex1: int = -1; // which normal on the poly found the shallowest depth?
		var separationMax1: Number = -Number.MAX_VALUE; // the shallowest depth of edge in poly
		var separation2: Number;
		var separationIndex2: int = -1; // which normal on the poly found the shallowest depth?
		var separationMax2: Number = -Number.MAX_VALUE; // the shallowest depth of edge in poly
		var separationMax: Number = -Number.MAX_VALUE; // the shallowest depth of edge in poly
		var separationV1: Boolean = false; // is the shallowest depth from edge's v1 or v2 vertex?
		var separationIndex: int = -1; // which normal on the poly found the shallowest depth?
		
		var vertexCount: int = polygon.m_vertexCount;
		var vertices: Array = polygon.m_vertices;
		var normals: Array = polygon.m_normals;
		
		var enterStartIndex: int = -1; // the last poly vertex above the edge
		var enterEndIndex: int = -1; // the first poly vertex below the edge
		var exitStartIndex: int = -1; // the last poly vertex below the edge
		var exitEndIndex: int = -1; // the first poly vertex above the edge
		
		// the "N" in the following variables refers to the edge's normal. 
		// these are projections of poly vertices along the edge's normal, 
		// a.k.a. they are the separation of the poly from the edge. 
		var prevSepN: Number = 0.0;
		var nextSepN: Number = 0.0;
		var enterSepN: Number = 0.0; // the depth of enterEndIndex under the edge (stored as a separation, so it's negative)
		var exitSepN: Number = 0.0; // the depth of exitStartIndex under the edge (stored as a separation, so it's negative)
		var deepestSepN: Number = Number.MAX_VALUE; // the depth of the deepest poly vertex under the end (stored as a separation, so it's negative)
		
		
		// for each poly normal, get the edge's depth into the poly. 
		// for each poly vertex, get the vertex's depth into the edge. 
		// use these calculations to define the remaining variables declared above.
		tVec1 = vertices[vertexCount-1];
		prevSepN = (tVec1.x - v1LocalX) * nLocalX + (tVec1.y - v1LocalY) * nLocalY;
		for (var i: int = 0; i < vertexCount; i++)
		{
			tVec1 = vertices[i];
			tVec2 = normals[i];
			separation1 = (v1LocalX - tVec1.x) * tVec2.x + (v1LocalY - tVec1.y) * tVec2.y;
			separation2 = (v2LocalX - tVec1.x) * tVec2.x + (v2LocalY - tVec1.y) * tVec2.y;
			if (separation2 < separation1) {
				if (separation2 > separationMax) {
					separationMax = separation2;
					separationV1 = false;
					separationIndex = i;
				}
			} else {
				if (separation1 > separationMax) {
					separationMax = separation1;
					separationV1 = true;
					separationIndex = i;
				}
			}
			if (separation1 > separationMax1) {
				separationMax1 = separation1;
				separationIndex1 = i;
			}
			if (separation2 > separationMax2) {
				separationMax2 = separation2;
				separationIndex2 = i;
			}
			
			nextSepN = (tVec1.x - v1LocalX) * nLocalX + (tVec1.y - v1LocalY) * nLocalY;
			if (nextSepN >= 0.0 && prevSepN < 0.0) {
				exitStartIndex = (i == 0) ? vertexCount-1 : i-1;
				exitEndIndex = i;
				exitSepN = prevSepN;
			} else if (nextSepN < 0.0 && prevSepN >= 0.0) {
				enterStartIndex = (i == 0) ? vertexCount-1 : i-1;
				enterEndIndex = i;
				enterSepN = nextSepN;
			}
			if (nextSepN < deepestSepN) {
				deepestSepN = nextSepN;
			}
			prevSepN = nextSepN;
		}
		
		if (enterStartIndex == -1) {
			// poly is entirely below or entirely above edge, return with no contact:
			return;
		}
		if (separationMax > 0.0) {
			// poly is laterally disjoint with edge, return with no contact:
			return;
		}
		
		// if the poly is near a convex corner on the edge
		if ((separationV1 && edge.m_cornerConvex1) || (!separationV1 && edge.m_cornerConvex2)) {
			// if shallowest depth was from edge into poly, 
			// use the edge's vertex as the contact point:
			if (separationMax > deepestSepN + b2Settings.b2_linearSlop) {
				// if -normal angle is closer to adjacent edge than this edge, 
				// let the adjacent edge handle it and return with no contact:
				if (separationV1) {
					tMat = xf2.R;
					tVec1 = edge.m_cornerDir1;
					tX = (tMat.col1.x * tVec1.x + tMat.col2.x * tVec1.y);
					tY = (tMat.col1.y * tVec1.x + tMat.col2.y * tVec1.y);
					tMat = xf1.R;
					v1X = (tMat.col1.x * tX + tMat.col2.x * tY); // note abuse of v1... 
					v1Y = (tMat.col1.y * tX + tMat.col2.y * tY);
					tVec2 = normals[separationIndex1];
					if (tVec2.x * v1X + tVec2.y * v1Y >= 0.0) {
						return;
					}
				} else {
					tMat = xf2.R;
					tVec1 = edge.m_cornerDir2;
					tX = (tMat.col1.x * tVec1.x + tMat.col2.x * tVec1.y);
					tY = (tMat.col1.y * tVec1.x + tMat.col2.y * tVec1.y);
					tMat = xf1.R;
					v1X = (tMat.col1.x * tX + tMat.col2.x * tY); // note abuse of v1... 
					v1Y = (tMat.col1.y * tX + tMat.col2.y * tY);
					tVec2 = normals[separationIndex2];
					if (tVec2.x * v1X + tVec2.y * v1Y <= 0.0) {
						return;
					}
				}
				
				tPoint = manifold.points[0];
				manifold.pointCount = 1;
				
				//manifold->normal = b2Mul(xf1.R, normals[separationIndex]);
				tMat = xf1.R;
				tVec2 = normals[separationIndex];
				manifold.normal.x = (tMat.col1.x * tVec2.x + tMat.col2.x * tVec2.y);
				manifold.normal.y = (tMat.col1.y * tVec2.x + tMat.col2.y * tVec2.y);
				
				tPoint.separation = separationMax;
				tPoint.id.features.incidentEdge = separationIndex;
				tPoint.id.features.incidentVertex = b2Collision.b2_nullFeature;
				tPoint.id.features.referenceEdge = 0;
				tPoint.id.features.flip = 0;
				if (separationV1) {
					tPoint.localPoint1.x = v1LocalX;
					tPoint.localPoint1.y = v1LocalY;
					tPoint.localPoint2.x = edge.m_v1.x;
					tPoint.localPoint2.y = edge.m_v1.y;
				} else {
					tPoint.localPoint1.x = v2LocalX;
					tPoint.localPoint1.y = v2LocalY;
					tPoint.localPoint2.x = edge.m_v2.x;
					tPoint.localPoint2.y = edge.m_v2.y;
				}
				return;
			}
		}
		
		// We're going to use the edge's normal now.
		manifold.normal.x = -nX;
		manifold.normal.y = -nY;
		
		// Check whether we only need one contact point.
		if (enterEndIndex == exitStartIndex) {
			tPoint = manifold.points[0];
			manifold.pointCount = 1;
			tPoint.id.features.incidentEdge = enterEndIndex;
			tPoint.id.features.incidentVertex = b2Collision.b2_nullFeature;
			tPoint.id.features.referenceEdge = 0;
			tPoint.id.features.flip = 0;
			tVec1 = vertices[enterEndIndex];
			tPoint.localPoint1.x = tVec1.x;
			tPoint.localPoint1.y = tVec1.y;
			
			tMat = xf1.R;
			tX = xf1.position.x + (tMat.col1.x * tVec1.x + tMat.col2.x * tVec1.y) - xf2.position.x;
			tY = xf1.position.y + (tMat.col1.y * tVec1.x + tMat.col2.y * tVec1.y) - xf2.position.y;
			tMat = xf2.R;
			tPoint.localPoint2.x = (tX * tMat.col1.x + tY * tMat.col1.y );
			tPoint.localPoint2.y = (tX * tMat.col2.x + tY * tMat.col2.y );
			
			tPoint.separation = enterSepN;
			return;
		}
			
		manifold.pointCount = 2;
		
		// the edge's direction vector, but in the frame of the polygon:
		tX = -nLocalY;
		tY = nLocalX;
		
		tVec1 = vertices[enterEndIndex];
		var dirProj1: Number = tX * (tVec1.x - v1LocalX) + tY * (tVec1.y - v1LocalY);
		var dirProj2: Number;
		
		// The contact resolution is more robust if the two manifold points are 
		// adjacent to each other on the polygon. So pick the first two poly
		// vertices that are under the edge:
		exitEndIndex = (enterEndIndex == vertexCount - 1) ? 0 : enterEndIndex + 1;
		tVec1 = vertices[exitStartIndex];
		if (exitEndIndex != exitStartIndex) {
			exitStartIndex = exitEndIndex;
			
			exitSepN = nLocalX * (tVec1.x - v1LocalX) + nLocalY * (tVec1.y - v1LocalY);
		}
		dirProj2 = tX * (tVec1.x - v1LocalX) + tY * (tVec1.y - v1LocalY);
		
		tPoint = manifold.points[0];
		tPoint.id.features.incidentEdge = enterEndIndex;
		tPoint.id.features.incidentVertex = b2Collision.b2_nullFeature;
		tPoint.id.features.referenceEdge = 0;
		tPoint.id.features.flip = 0;
		
		if (dirProj1 > edge.m_length) {
			tPoint.localPoint1.x = v2LocalX;
			tPoint.localPoint1.y = v2LocalY;
			tPoint.localPoint2.x = edge.m_v2.x;
			tPoint.localPoint2.y = edge.m_v2.y;
			ratio = (edge.m_length - dirProj2) / (dirProj1 - dirProj2);
			if (ratio > 100.0 * Number.MIN_VALUE && ratio < 1.0) {
				tPoint.separation = exitSepN * (1.0 - ratio) + enterSepN * ratio;
			} else {
				tPoint.separation = enterSepN;
			}
		} else {
			tVec1 = vertices[enterEndIndex];
			tPoint.localPoint1.x = tVec1.x;
			tPoint.localPoint1.y = tVec1.y;
			
			tMat = xf1.R;
			tX = xf1.position.x + (tMat.col1.x * tVec1.x + tMat.col2.x * tVec1.y) - xf2.position.x;
			tY = xf1.position.y + (tMat.col1.y * tVec1.x + tMat.col2.y * tVec1.y) - xf2.position.y;
			tMat = xf2.R;
			tPoint.localPoint2.x = (tX * tMat.col1.x + tY * tMat.col1.y);
			tPoint.localPoint2.y = (tX * tMat.col2.x + tY * tMat.col2.y);
			
			tPoint.separation = enterSepN;
		}
		
		tPoint = manifold.points[1];
		tPoint.id.features.incidentEdge = exitStartIndex;
		tPoint.id.features.incidentVertex = b2Collision.b2_nullFeature;
		tPoint.id.features.referenceEdge = 0;
		tPoint.id.features.flip = 0;
		
		if (dirProj2 < 0.0) {
			tPoint.localPoint1.x = v1LocalX;
			tPoint.localPoint1.y = v1LocalY;
			tPoint.localPoint2.x = edge.m_v1.x;
			tPoint.localPoint2.y = edge.m_v1.y;
			ratio = (-dirProj1) / (dirProj2 - dirProj1);
			if (ratio > 100.0 * Number.MIN_VALUE && ratio < 1.0) {
				tPoint.separation = enterSepN * (1.0 - ratio) + exitSepN * ratio;
			} else {
				tPoint.separation = exitSepN;
			}
		} else {
			tVec1 = vertices[exitStartIndex];
			tPoint.localPoint1.x = tVec1.x;
			tPoint.localPoint1.y = tVec1.y;
			
			tMat = xf1.R;
			tX = xf1.position.x + (tMat.col1.x * tVec1.x + tMat.col2.x * tVec1.y) - xf2.position.x;
			tY = xf1.position.y + (tMat.col1.y * tVec1.x + tMat.col2.y * tVec1.y) - xf2.position.y;
			tMat = xf2.R;
			tPoint.localPoint2.x = (tX * tMat.col1.x + tY * tMat.col1.y);
			tPoint.localPoint2.y = (tX * tMat.col2.x + tY * tMat.col2.y);
			
			tPoint.separation = exitSepN;
		}
	}
	
	public override function GetManifolds():Array
	{
		return m_manifolds;
	}

	private var m_manifolds:Array = [new b2Manifold()];
	private var m_manifold:b2Manifold;
	private var m0:b2Manifold = new b2Manifold();
	
}

}