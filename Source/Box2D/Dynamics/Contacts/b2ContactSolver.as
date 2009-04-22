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

	
import Box2D.Collision.Shapes.b2Shape;
import Box2D.Dynamics.*;
import Box2D.Collision.*;
import Box2D.Common.Math.*;
import Box2D.Common.*;
import Box2D.Dynamics.Contacts.*;

import Box2D.Common.b2internal;
use namespace b2internal;


/**
* @private
*/
public class b2ContactSolver
{
	public function b2ContactSolver(step:b2TimeStep, contacts:Array, contactCount:int, allocator:*){
		var contact:b2Contact;
		
		//m_step = step;
		m_step.dt = step.dt;
		m_step.inv_dt = step.inv_dt;
		m_step.positionIterations = step.positionIterations;
		m_step.velocityIterations = step.velocityIterations;
		
		m_allocator = allocator;
		
		var i:int;
		var tVec:b2Vec2;
		var tMat:b2Mat22;
		
		m_constraintCount = 0;
		for (i = 0; i < contactCount; ++i)
		{
			// b2Assert(contacts[i].IsSolid());
			contact = contacts[i];
			m_constraintCount += contact.m_manifoldCount;
		}
		
		// fill array
		for (i = 0; i < m_constraintCount; i++){
			m_constraints[i] = new b2ContactConstraint();
		}
		
		var count:int = 0;
		for (i = 0; i < contactCount; ++i)
		{
			contact = contacts[i];
			var shape1:b2Shape = contact.m_shape1;
			var shape2:b2Shape = contact.m_shape2;
			var b1:b2Body = shape1.m_body;
			var b2:b2Body = shape2.m_body;
			var manifoldCount:int = contact.m_manifoldCount;
			var manifolds:Array = contact.GetManifolds();
			var friction:Number = b2Settings.b2MixFriction(shape1.GetFriction(), shape2.GetFriction());
			var restitution:Number = b2Settings.b2MixRestitution(shape1.GetRestitution(), shape2.GetRestitution());
			
			//var v1:b2Vec2 = b1.m_linearVelocity.Copy();
			var v1X:Number = b1.m_linearVelocity.x;
			var v1Y:Number = b1.m_linearVelocity.y;
			//var v2:b2Vec2 = b2.m_linearVelocity.Copy();
			var v2X:Number = b2.m_linearVelocity.x;
			var v2Y:Number = b2.m_linearVelocity.y;
			var w1:Number = b1.m_angularVelocity;
			var w2:Number = b2.m_angularVelocity;
			
			for (var j:int = 0; j < manifoldCount; ++j)
			{
				var manifold:b2Manifold = manifolds[ j ];
				
				//b2Settings.b2Assert(manifold.pointCount > 0);
				
				//var normal:b2Vec2 = manifold.normal.Copy();
				var normalX:Number = manifold.normal.x;
				var normalY:Number = manifold.normal.y;
				
				//b2Settings.b2Assert(count < m_constraintCount);
				var cc:b2ContactConstraint = m_constraints[ count ];
				cc.body1 = b1; //p
				cc.body2 = b2; //p
				cc.manifold = manifold; //p
				//c.normal = normal;
				cc.normal.x = normalX;
				cc.normal.y = normalY;
				cc.pointCount = manifold.pointCount;
				cc.friction = friction;
				cc.restitution = restitution;
				
				for (var k:uint = 0; k < cc.pointCount; ++k)
				{
					var cp:b2ManifoldPoint = manifold.points[ k ];
					var ccp:b2ContactConstraintPoint = cc.points[ k ];
					
					ccp.normalImpulse = cp.normalImpulse;
					ccp.tangentImpulse = cp.tangentImpulse;
					ccp.separation = cp.separation;
					
					ccp.localAnchor1.SetV(cp.localPoint1);
					ccp.localAnchor2.SetV(cp.localPoint2);
					
					var tX:Number;
					var tY:Number;
					
					//ccp->r1 = b2Mul(b1->GetXForm().R, cp->localPoint1 - b1->GetLocalCenter());
					tMat = b1.m_xf.R;
					var r1X:Number = cp.localPoint1.x - b1.m_sweep.localCenter.x;
					var r1Y:Number = cp.localPoint1.y - b1.m_sweep.localCenter.y;
					tX  = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
					r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
					r1X = tX;
					ccp.r1.Set(r1X,r1Y);
					//ccp->r2 = b2Mul(b2->GetXForm().R, cp->localPoint2 - b2->GetLocalCenter());
					tMat = b2.m_xf.R;
					var r2X:Number = cp.localPoint2.x - b2.m_sweep.localCenter.x;
					var r2Y:Number = cp.localPoint2.y - b2.m_sweep.localCenter.y;
					tX  = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
					r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
					r2X = tX;
					ccp.r2.Set(r2X,r2Y);
					
					var rn1:Number = r1X * normalY - r1Y * normalX;//b2Math.b2Cross(r1, normal);
					var rn2:Number = r2X * normalY - r2Y * normalX;//b2Math.b2Cross(r2, normal);
					
					rn1 *= rn1;
					rn2 *= rn2;
					
					var kNormal:Number = b1.m_invMass + b2.m_invMass + b1.m_invI * rn1 + b2.m_invI * rn2;
					//b2Settings.b2Assert(kNormal > Number.MIN_VALUE);
					ccp.normalMass = 1.0 / kNormal;
					
					var kEqualized:Number = b1.m_mass * b1.m_invMass + b2.m_mass * b2.m_invMass;
					kEqualized += b1.m_mass * b1.m_invI * rn1 + b2.m_mass * b2.m_invI * rn2;
					//b2Assert(kEqualized > Number.MIN_VALUE);
					ccp.equalizedMass = 1.0 / kEqualized;
					
					//var tangent:b2Vec2 = b2Math.b2CrossVF(normal, 1.0);
					var tangentX:Number = normalY
					var tangentY:Number = -normalX;
					
					//var rt1:Number = b2Math.b2Cross(r1, tangent);
					var rt1:Number = r1X*tangentY - r1Y*tangentX;
					//var rt2:Number = b2Math.b2Cross(r2, tangent);
					var rt2:Number = r2X*tangentY - r2Y*tangentX;
					
					rt1 *= rt1;
					rt2 *= rt2;
					
					var kTangent:Number = b1.m_invMass + b2.m_invMass + b1.m_invI * rt1 + b2.m_invI * rt2;
					//b2Settings.b2Assert(kTangent > Number.MIN_VALUE);
					ccp.tangentMass = 1.0 /  kTangent;
					
					// Setup a velocity bias for restitution.
					ccp.velocityBias = 0.0;
					if (ccp.separation > 0.0)
					{
						ccp.velocityBias = -60.0 * ccp.separation; // TODO_ERIN b2TimeStep
					}
					else
					{
						//b2Dot(c.normal, v2 + b2Cross(w2, r2) - v1 - b2Cross(w1, r1));
						tX = v2X + (-w2*r2Y) - v1X - (-w1*r1Y);
						tY = v2Y + (w2*r2X) - v1Y - (w1*r1X);
						//var vRel:Number = b2Dot(cc.normal, t);
						var vRel:Number = cc.normal.x*tX + cc.normal.y*tY;
						if (vRel < -b2Settings.b2_velocityThreshold)
						{
							ccp.velocityBias += -cc.restitution * vRel;
						}
					}
				}
				
				// If we have two points, then prepare the block solver.
				if (cc.pointCount == 2)
				{
					var ccp1:b2ContactConstraintPoint = cc.points[0];
					var ccp2:b2ContactConstraintPoint = cc.points[1];
					
					var invMass1:Number = b1.m_invMass;
					var invI1:Number = b1.m_invI;
					var invMass2:Number = b2.m_invMass;
					var invI2:Number = b2.m_invI;
					
					//var rn11:Number = b2Cross(ccp1.r1, normal);
					//var rn12:Number = b2Cross(ccp1.r2, normal);
					//var rn21:Number = b2Cross(ccp2.r1, normal);
					//var rn22:Number = b2Cross(ccp2.r2, normal);
					var rn11:Number = ccp1.r1.x * normalY - ccp1.r1.y * normalX;
					var rn12:Number = ccp1.r2.x * normalY - ccp1.r2.y * normalX;
					var rn21:Number = ccp2.r1.x * normalY - ccp2.r1.y * normalX;
					var rn22:Number = ccp2.r2.x * normalY - ccp2.r2.y * normalX;
					
					var k11:Number = invMass1 + invMass2 + invI1 * rn11 * rn11 + invI2 * rn12 * rn12;
					var k22:Number = invMass1 + invMass2 + invI1 * rn21 * rn21 + invI2 * rn22 * rn22;
					var k12:Number = invMass1 + invMass2 + invI1 * rn11 * rn21 + invI2 * rn12 * rn22;
					
					// Ensure a reasonable condition number.
					var k_maxConditionNumber:Number = 100.0;
					if ( k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12))
					{
						// K is safe to invert.
						cc.K.col1.Set(k11, k12);
						cc.K.col2.Set(k12, k22);
						cc.K.GetInverse(cc.normalMass);
					}
					else
					{
						// The constraints are redundant, just use one.
						// TODO_ERIN use deepest?
						cc.pointCount = 1;
					}
				}
				
				++count;
			}
		}
		
		//b2Settings.b2Assert(count == m_constraintCount);
	}
	//~b2ContactSolver();

	public function InitVelocityConstraints(step: b2TimeStep) : void{
		var tVec:b2Vec2;
		var tVec2:b2Vec2;
		var tMat:b2Mat22;
		
		// Warm start.
		for (var i:int = 0; i < m_constraintCount; ++i)
		{
			var c:b2ContactConstraint = m_constraints[ i ];
			
			var b1:b2Body = c.body1;
			var b2:b2Body = c.body2;
			var invMass1:Number = b1.m_invMass;
			var invI1:Number = b1.m_invI;
			var invMass2:Number = b2.m_invMass;
			var invI2:Number = b2.m_invI;
			//var normal:b2Vec2 = new b2Vec2(c.normal.x, c.normal.y);
			var normalX:Number = c.normal.x;
			var normalY:Number = c.normal.y;
			//var tangent:b2Vec2 = b2Math.b2CrossVF(normal, 1.0);
			var tangentX:Number = normalY;
			var tangentY:Number = -normalX;
			
			var tX:Number;
			
			var j:int;
			var tCount:int;
			if (step.warmStarting)
			{
				tCount = c.pointCount;
				for (j = 0; j < tCount; ++j)
				{
					var ccp:b2ContactConstraintPoint = c.points[ j ];
					ccp.normalImpulse *= step.dtRatio;
					ccp.tangentImpulse *= step.dtRatio;
					//b2Vec2 P = ccp->normalImpulse * normal + ccp->tangentImpulse * tangent;
					var PX:Number = ccp.normalImpulse * normalX + ccp.tangentImpulse * tangentX;
					var PY:Number = ccp.normalImpulse * normalY + ccp.tangentImpulse * tangentY;
					
					//b1.m_angularVelocity -= invI1 * b2Math.b2CrossVV(r1, P);
					b1.m_angularVelocity -= invI1 * (ccp.r1.x * PY - ccp.r1.y * PX);
					//b1.m_linearVelocity.Subtract( b2Math.MulFV(invMass1, P) );
					b1.m_linearVelocity.x -= invMass1 * PX;
					b1.m_linearVelocity.y -= invMass1 * PY;
					//b2.m_angularVelocity += invI2 * b2Math.b2CrossVV(r2, P);
					b2.m_angularVelocity += invI2 * (ccp.r2.x * PY - ccp.r2.y * PX);
					//b2.m_linearVelocity.Add( b2Math.MulFV(invMass2, P) );
					b2.m_linearVelocity.x += invMass2 * PX;
					b2.m_linearVelocity.y += invMass2 * PY;
				}
			}
			else{
				tCount = c.pointCount;
				for (j = 0; j < tCount; ++j)
				{
					var ccp2:b2ContactConstraintPoint = c.points[ j ];
					ccp2.normalImpulse = 0.0;
					ccp2.tangentImpulse = 0.0;
				}
			}
		}
	}
	public function SolveVelocityConstraints() : void{
		var j:int;
		var ccp:b2ContactConstraintPoint;
		var r1X:Number;
		var r1Y:Number;
		var r2X:Number;
		var r2Y:Number;
		var dvX:Number;
		var dvY:Number;
		var vn:Number;
		var vt:Number;
		var lambda_n:Number;
		var lambda_t:Number;
		var newImpulse_n:Number;
		var newImpulse_t:Number;
		var PX:Number;
		var PY:Number;
		var dX:Number;
		var dY:Number;
		var P1X:Number;
		var P1Y:Number;
		var P2X:Number;
		var P2Y:Number;
		
		var tMat:b2Mat22;
		var tVec:b2Vec2;
		
		for (var i:int = 0; i < m_constraintCount; ++i)
		{
			var c:b2ContactConstraint = m_constraints[ i ];
			var b1:b2Body = c.body1;
			var b2:b2Body = c.body2;
			var w1:Number = b1.m_angularVelocity;
			var w2:Number = b2.m_angularVelocity;
			var v1:b2Vec2 = b1.m_linearVelocity;
			var v2:b2Vec2 = b2.m_linearVelocity;
			
			var invMass1:Number = b1.m_invMass;
			var invI1:Number = b1.m_invI;
			var invMass2:Number = b2.m_invMass;
			var invI2:Number = b2.m_invI;
			//var normal:b2Vec2 = new b2Vec2(c.normal.x, c.normal.y);
			var normalX:Number = c.normal.x;
			var normalY:Number = c.normal.y;
			//var tangent:b2Vec2 = b2Math.b2CrossVF(normal, 1.0);
			var tangentX:Number = normalY;
			var tangentY:Number = -normalX;
			var friction:Number = c.friction;
			
			var tX:Number;
			
			//b2Settings.b2Assert(c.pointCount == 1 || c.pointCount == 2);
			
			// Solve the normal constraints
			var tCount:int = c.pointCount;
			if (c.pointCount == 1)
			{
				ccp = c.points[ 0 ];
				
				// Relative velocity at contact
				//b2Vec2 dv = v2 + b2Cross(w2, ccp->r2) - v1 - b2Cross(w1, ccp->r1);
				dvX = v2.x + (-w2 * ccp.r2.y) - v1.x - (-w1 * ccp.r1.y);
				dvY = v2.y + (w2 * ccp.r2.x) - v1.y - (w1 * ccp.r1.x);
				
				// Compute normal impulse
				//var vn:Number = b2Math.b2Dot(dv, normal);
				vn = dvX * normalX + dvY * normalY;
				lambda_n = -ccp.normalMass * (vn - ccp.velocityBias);
				
				// Compute tangent impulse - normal
				vt = dvX*tangentX + dvY*tangentY;//b2Math.b2Dot(dv, tangent);
				lambda_t = ccp.tangentMass * (-vt);
				
				// b2Clamp the accumulated impulse - tangent
				newImpulse_n = b2Math.b2Max(ccp.normalImpulse + lambda_n, 0.0);
				lambda_n = newImpulse_n - ccp.normalImpulse;
				
				// b2Clamp the accumulated force
				var maxFriction:Number = friction * ccp.normalImpulse;
				newImpulse_t = b2Math.b2Clamp(ccp.tangentImpulse + lambda_t, -maxFriction, maxFriction);
				lambda_t = newImpulse_t - ccp.tangentImpulse;
				
				// Apply contact impulse
				//b2Vec2 P = lambda * normal;
				PX = lambda_n * normalX + lambda_t * tangentX;
				PY = lambda_n * normalY + lambda_t * tangentY;
				
				//v1.Subtract( b2Math.MulFV( invMass1, P ) );
				v1.x -= invMass1 * PX;
				v1.y -= invMass1 * PY;
				w1 -= invI1 * (ccp.r1.x * PY - ccp.r1.y * PX);//invI1 * b2Math.b2CrossVV(ccp.r1, P);
				
				//v2.Add( b2Math.MulFV( invMass2, P ) );
				v2.x += invMass2 * PX;
				v2.y += invMass2 * PY;
				w2 += invI2 * (ccp.r2.x * PY - ccp.r2.y * PX);//invI2 * b2Math.b2CrossVV(ccp.r2, P);
				
				ccp.normalImpulse = newImpulse_n;
				ccp.tangentImpulse = newImpulse_t;
			}
			else
			{
				// Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D_Lite).
				// Build the mini LCP for this contact patch
				//
				// vn = A * x + b, vn >= 0, , vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
				//
				// A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
				// b = vn_0 - velocityBias
				//
				// The system is solved using the "Total enumeration method" (s. Murty). The complementary constraint vn_i * x_i
				// implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D contact problem the cases
				// vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be tested. The first valid
				// solution that satisfies the problem is chosen.
				//
				// In order to account of the accumulated impulse 'a' (because of the iterative nature of the solver which only requires
				// that the accumulated impulse is clamped and not the incremental impulse) we change the impulse variable (x_i).
				//
				// Substitute:
				//
				// x = x' - a
				//
				// Plug into above equation:
				//
				// vn = A * x + b
				//    = A * (x' - a) + b
				//    = A * x' + b - A * a
				//    = A * x' + b'
				// b' = b - A * a;
				
				var cp1:b2ContactConstraintPoint = c.points[ 0 ];
				var cp2:b2ContactConstraintPoint = c.points[ 1 ];
				
				var aX:Number = cp1.normalImpulse;
				var aY:Number = cp2.normalImpulse;
				//b2Settings.b2Assert( aX >= 0.0f && aY >= 0.0f );
				
				// Relative velocity at contact
				//var dv1:b2Vec2 = v2 + b2Cross(w2, cp1.r2) - v1 - b2Cross(w1, cp1.r1);
				var dv1X:Number = v2.x - w2 * cp1.r2.y - v1.x + w1 * cp1.r1.y;
				var dv1Y:Number = v2.y + w2 * cp1.r2.x - v1.y - w1 * cp1.r1.x;
				//var dv2:b2Vec2 = v2 + b2Cross(w2, cp2.r2) - v1 - b2Cross(w1, cp2.r1);
				var dv2X:Number = v2.x - w2 * cp2.r2.y - v1.x + w1 * cp2.r1.y;
				var dv2Y:Number = v2.y + w2 * cp2.r2.x - v1.y - w1 * cp2.r1.x;
				
				// Compute normal velocity
				//var vn1:Number = b2Dot(dv1, normal);
				var vn1:Number = dv1X * normalX + dv1Y * normalY;
				//var vn2:Number = b2Dot(dv2, normal);
				var vn2:Number = dv2X * normalX + dv2Y * normalY;
				
				var bX:Number = vn1 - cp1.velocityBias;
				var bY:Number = vn2 - cp2.velocityBias;
				
				//b -= b2Mul(c.K,a);
				tMat = c.K;
				bX -= tMat.col1.x * aX + tMat.col2.x * aY;
				bY -= tMat.col1.y * aX + tMat.col2.y * aY;
				
				var k_errorTol:Number  = 0.001;
				for (;; )
				{
					//
					// Case 1: vn = 0
					//
					// 0 = A * x' + b'
					//
					// Solve for x':
					//
					// x' = -inv(A) * b'
					//
					
					//var x:b2Vec2 = - b2Mul(c->normalMass, b);
					tMat = c.normalMass;
					var xX:Number = - (tMat.col1.x * bX + tMat.col2.x * bY);
					var xY:Number = - (tMat.col1.y * bX + tMat.col2.y * bY);
					
					if (xX >= 0.0 && xY >= 0.0) {
						// Resubstitute for the incremental impulse
						//d = x - a;
						dX = xX - aX;
						dY = xY - aY;
						
						//Aply incremental impulse
						//P1 = d.x * normal;
						P1X = dX * normalX;
						P1Y = dX * normalY;
						//P2 = d.y * normal;
						P2X = dY * normalX;
						P2Y = dY * normalY;
						
						//v1 -= invMass1 * (P1 + P2)
						v1.x -= invMass1 * (P1X + P2X);
						v1.y -= invMass1 * (P1Y + P2Y);
						//w1 -= invI1 * (b2Cross(cp1.r1, P1) + b2Cross(cp2.r1, P2));
						w1 -= invI1 * ( cp1.r1.x * P1Y - cp1.r1.y * P1X + cp2.r1.x * P2Y - cp2.r1.y * P2X);
						
						//v2 += invMass2 * (P1 + P2)
						v2.x += invMass2 * (P1X + P2X);
						v2.y += invMass2 * (P1Y + P2Y);
						//w2 += invI2 * (b2Cross(cp1.r2, P1) + b2Cross(cp2.r2, P2));
						w2   += invI2 * ( cp1.r2.x * P1Y - cp1.r2.y * P1X + cp2.r2.x * P2Y - cp2.r2.y * P2X);
						
						// Accumulate
						cp1.normalImpulse = xX;
						cp2.normalImpulse = xY;
						
	//#if B2_DEBUG_SOLVER == 1
	//					// Post conditions
	//					//dv1 = v2 + b2Cross(w2, cp1.r2) - v1 - b2Cross(w1, cp1.r1);
	//					dv1X = v2.x - w2 * cp1.r2.y - v1.x + w1 * cp1.r1.y;
	//					dv1Y = v2.y + w2 * cp1.r2.x - v1.y - w1 * cp1.r1.x;
	//					//dv2 = v2 + b2Cross(w2, cp2.r2) - v1 - b2Cross(w1, cp2.r1);
	//					dv1X = v2.x - w2 * cp2.r2.y - v1.x + w1 * cp2.r1.y;
	//					dv1Y = v2.y + w2 * cp2.r2.x - v1.y - w1 * cp2.r1.x;
	//					// Compute normal velocity
	//					//vn1 = b2Dot(dv1, normal);
	//					vn1 = dv1X * normalX + dv1Y * normalY;
	//					//vn2 = b2Dot(dv2, normal);
	//					vn2 = dv2X * normalX + dv2Y * normalY;
	//
	//					//b2Settings.b2Assert(b2Abs(vn1 - cp1.velocityBias) < k_errorTol);
	//					//b2Settings.b2Assert(b2Abs(vn2 - cp2.velocityBias) < k_errorTol);
	//#endif
						break;
					}
					
					//
					// Case 2: vn1 = 0  and x2 = 0
					//
					//   0 = a11 * x1' + a12 * 0 + b1'
					// vn2 = a21 * x1' + a22 * 0 + b2'
					//
					
					xX = - cp1.normalMass * bX;
					xY = 0.0;
					vn1 = 0.0;
					vn2 = c.K.col1.y * xX + bY;
					
					if (xX >= 0.0 && vn2 >= 0.0)
					{
						// Resubstitute for the incremental impulse
						//d = x - a;
						dX = xX - aX;
						dY = xY - aY;
						
						//Aply incremental impulse
						//P1 = d.x * normal;
						P1X = dX * normalX;
						P1Y = dX * normalY;
						//P2 = d.y * normal;
						P2X = dY * normalX;
						P2Y = dY * normalY;
						
						//v1 -= invMass1 * (P1 + P2)
						v1.x -= invMass1 * (P1X + P2X);
						v1.y -= invMass1 * (P1Y + P2Y);
						//w1 -= invI1 * (b2Cross(cp1.r1, P1) + b2Cross(cp2.r1, P2));
						w1 -= invI1 * ( cp1.r1.x * P1Y - cp1.r1.y * P1X + cp2.r1.x * P2Y - cp2.r1.y * P2X);
						
						//v2 += invMass2 * (P1 + P2)
						v2.x += invMass2 * (P1X + P2X);
						v2.y += invMass2 * (P1Y + P2Y);
						//w2 += invI2 * (b2Cross(cp1.r2, P1) + b2Cross(cp2.r2, P2));
						w2   += invI2 * ( cp1.r2.x * P1Y - cp1.r2.y * P1X + cp2.r2.x * P2Y - cp2.r2.y * P2X);
						
						// Accumulate
						cp1.normalImpulse = xX;
						cp2.normalImpulse = xY;
						
	//#if B2_DEBUG_SOLVER == 1
	//					// Post conditions
	//					//dv1 = v2 + b2Cross(w2, cp1.r2) - v1 - b2Cross(w1, cp1.r1);
	//					dv1X = v2.x - w2 * cp1.r2.y - v1.x + w1 * cp1.r1.y;
	//					dv1Y = v2.y + w2 * cp1.r2.x - v1.y - w1 * cp1.r1.x;
	//					//dv2 = v2 + b2Cross(w2, cp2.r2) - v1 - b2Cross(w1, cp2.r1);
	//					dv1X = v2.x - w2 * cp2.r2.y - v1.x + w1 * cp2.r1.y;
	//					dv1Y = v2.y + w2 * cp2.r2.x - v1.y - w1 * cp2.r1.x;
	//					// Compute normal velocity
	//					//vn1 = b2Dot(dv1, normal);
	//					vn1 = dv1X * normalX + dv1Y * normalY;
	//					//vn2 = b2Dot(dv2, normal);
	//					vn2 = dv2X * normalX + dv2Y * normalY;
	//
	//					//b2Settings.b2Assert(b2Abs(vn1 - cp1.velocityBias) < k_errorTol);
	//					//b2Settings.b2Assert(b2Abs(vn2 - cp2.velocityBias) < k_errorTol);
	//#endif
						break;
					}
					
					//
					// Case 3: w2 = 0 and x1 = 0
					//
					// vn1 = a11 * 0 + a12 * x2' + b1'
					//   0 = a21 * 0 + a22 * x2' + b2'
					//
					
					xX = 0.0;
					xY = -cp2.normalMass * bY;
					vn1 = c.K.col2.x * xY + bX;
					vn2 = 0.0;
					if (xY >= 0.0 && vn1 >= 0.0)
					{
						// Resubstitute for the incremental impulse
						//d = x - a;
						dX = xX - aX;
						dY = xY - aY;
						
						//Aply incremental impulse
						//P1 = d.x * normal;
						P1X = dX * normalX;
						P1Y = dX * normalY;
						//P2 = d.y * normal;
						P2X = dY * normalX;
						P2Y = dY * normalY;
						
						//v1 -= invMass1 * (P1 + P2)
						v1.x -= invMass1 * (P1X + P2X);
						v1.y -= invMass1 * (P1Y + P2Y);
						//w1 -= invI1 * (b2Cross(cp1.r1, P1) + b2Cross(cp2.r1, P2));
						w1 -= invI1 * ( cp1.r1.x * P1Y - cp1.r1.y * P1X + cp2.r1.x * P2Y - cp2.r1.y * P2X);
						
						//v2 += invMass2 * (P1 + P2)
						v2.x += invMass2 * (P1X + P2X);
						v2.y += invMass2 * (P1Y + P2Y);
						//w2 += invI2 * (b2Cross(cp1.r2, P1) + b2Cross(cp2.r2, P2));
						w2   += invI2 * ( cp1.r2.x * P1Y - cp1.r2.y * P1X + cp2.r2.x * P2Y - cp2.r2.y * P2X);
						
						// Accumulate
						cp1.normalImpulse = xX;
						cp2.normalImpulse = xY;
						
	//#if B2_DEBUG_SOLVER == 1
	//					// Post conditions
	//					//dv1 = v2 + b2Cross(w2, cp1.r2) - v1 - b2Cross(w1, cp1.r1);
	//					dv1X = v2.x - w2 * cp1.r2.y - v1.x + w1 * cp1.r1.y;
	//					dv1Y = v2.y + w2 * cp1.r2.x - v1.y - w1 * cp1.r1.x;
	//					//dv2 = v2 + b2Cross(w2, cp2.r2) - v1 - b2Cross(w1, cp2.r1);
	//					dv1X = v2.x - w2 * cp2.r2.y - v1.x + w1 * cp2.r1.y;
	//					dv1Y = v2.y + w2 * cp2.r2.x - v1.y - w1 * cp2.r1.x;
	//					// Compute normal velocity
	//					//vn1 = b2Dot(dv1, normal);
	//					vn1 = dv1X * normalX + dv1Y * normalY;
	//					//vn2 = b2Dot(dv2, normal);
	//					vn2 = dv2X * normalX + dv2Y * normalY;
	//
	//					//b2Settings.b2Assert(b2Abs(vn1 - cp1.velocityBias) < k_errorTol);
	//					//b2Settings.b2Assert(b2Abs(vn2 - cp2.velocityBias) < k_errorTol);
	//#endif
						break;
					}
					
					//
					// Case 4: x1 = 0 and x2 = 0
					//
					// vn1 = b1
					// vn2 = b2
					
					xX = 0.0;
					xY = 0.0;
					vn1 = bX;
					vn2 = bY;
					
					if (vn1 >= 0.0 && vn2 >= 0.0 ) {
						// Resubstitute for the incremental impulse
						//d = x - a;
						dX = xX - aX;
						dY = xY - aY;
						
						//Aply incremental impulse
						//P1 = d.x * normal;
						P1X = dX * normalX;
						P1Y = dX * normalY;
						//P2 = d.y * normal;
						P2X = dY * normalX;
						P2Y = dY * normalY;
						
						//v1 -= invMass1 * (P1 + P2)
						v1.x -= invMass1 * (P1X + P2X);
						v1.y -= invMass1 * (P1Y + P2Y);
						//w1 -= invI1 * (b2Cross(cp1.r1, P1) + b2Cross(cp2.r1, P2));
						w1 -= invI1 * ( cp1.r1.x * P1Y - cp1.r1.y * P1X + cp2.r1.x * P2Y - cp2.r1.y * P2X);
						
						//v2 += invMass2 * (P1 + P2)
						v2.x += invMass2 * (P1X + P2X);
						v2.y += invMass2 * (P1Y + P2Y);
						//w2 += invI2 * (b2Cross(cp1.r2, P1) + b2Cross(cp2.r2, P2));
						w2   += invI2 * ( cp1.r2.x * P1Y - cp1.r2.y * P1X + cp2.r2.x * P2Y - cp2.r2.y * P2X);
						
						// Accumulate
						cp1.normalImpulse = xX;
						cp2.normalImpulse = xY;
						
	//#if B2_DEBUG_SOLVER == 1
	//					// Post conditions
	//					//dv1 = v2 + b2Cross(w2, cp1.r2) - v1 - b2Cross(w1, cp1.r1);
	//					dv1X = v2.x - w2 * cp1.r2.y - v1.x + w1 * cp1.r1.y;
	//					dv1Y = v2.y + w2 * cp1.r2.x - v1.y - w1 * cp1.r1.x;
	//					//dv2 = v2 + b2Cross(w2, cp2.r2) - v1 - b2Cross(w1, cp2.r1);
	//					dv1X = v2.x - w2 * cp2.r2.y - v1.x + w1 * cp2.r1.y;
	//					dv1Y = v2.y + w2 * cp2.r2.x - v1.y - w1 * cp2.r1.x;
	//					// Compute normal velocity
	//					//vn1 = b2Dot(dv1, normal);
	//					vn1 = dv1X * normalX + dv1Y * normalY;
	//					//vn2 = b2Dot(dv2, normal);
	//					vn2 = dv2X * normalX + dv2Y * normalY;
	//
	//					//b2Settings.b2Assert(b2Abs(vn1 - cp1.velocityBias) < k_errorTol);
	//					//b2Settings.b2Assert(b2Abs(vn2 - cp2.velocityBias) < k_errorTol);
	//#endif
						break;
					}
					
					// No solution, give up. This is hit sometimes, but it doesn't seem to matter.
					break;
				}
				
				// Solve tangent constraints
				for (j = 0; j < c.pointCount; ++j)
				{
					ccp = c.points[j];
					
					// Relative velocity at contact
					//dv = v2 + b2Cross(w2, ccp.r2) - v1 - b2Cross(w1, ccp.r1);
					dvX = v2.x - w2 * ccp.r2.y - v1.x + w1 * ccp.r1.y;
					dvY = v2.y + w2 * ccp.r2.x - v1.y - w1 * ccp.r1.x;
					
					// Compute tangent force
					vt = dvX * tangentX + dvY * tangentY;
					lambda_t = ccp.tangentMass * ( -vt);
					
					// b2Clamp the accumulated force
					maxFriction = friction * ccp.normalImpulse;
					newImpulse_t = b2Math.b2Clamp(ccp.tangentImpulse + lambda_t, -maxFriction, maxFriction);
					lambda_t = newImpulse_t - ccp.tangentImpulse;
					
					// Apply contact impulse
					PX = lambda_t * tangentX;
					PY = lambda_t * tangentY;
					
					v1.x -= invMass1 * PX;
					v1.y -= invMass1 * PY;
					w1   -= invI1 * (ccp.r1.x * PY - ccp.r1.y * PX);
					
					v2.x += invMass2 * PX;
					v2.y += invMass2 * PY;
					w2   += invI2 * (ccp.r2.x * PY - ccp.r2.y * PX);
					
					ccp.tangentImpulse = newImpulse_t;
				}
			}
			
			
			// b2Vec2s in AS3 are copied by reference. The originals are 
			// references to the same things here and there is no need to 
			// copy them back, unlike in C++ land where b2Vec2s are 
			// copied by value.
			/*b1->m_linearVelocity = v1;
			b2->m_linearVelocity = v2;*/
			b1.m_angularVelocity = w1;
			b2.m_angularVelocity = w2;
		}
	}
	
	public function FinalizeVelocityConstraints() : void
	{
		for (var i:int = 0; i < m_constraintCount; ++i)
		{
			var c:b2ContactConstraint = m_constraints[ i ];
			var m:b2Manifold = c.manifold;
			
			for (var j:int = 0; j < c.pointCount; ++j)
			{
				var point1:b2ManifoldPoint = m.points[j];
				var point2:b2ContactConstraintPoint = c.points[j];
				point1.normalImpulse = point2.normalImpulse;
				point1.tangentImpulse = point2.tangentImpulse;
			}
		}
	}
	
//#if 1
// Sequential solver
	public function SolvePositionConstraints(baumgarte:Number):Boolean{
		var minSeparation:Number = 0.0;
		
		var tMat:b2Mat22;
		var tVec:b2Vec2;
		
		for (var i:int = 0; i < m_constraintCount; ++i)
		{
			var c:b2ContactConstraint = m_constraints[ i ];
			var b1:b2Body = c.body1;
			var b2:b2Body = c.body2;
			var b1_sweep_c:b2Vec2 = b1.m_sweep.c;
			var b1_sweep_a:Number = b1.m_sweep.a;
			var b2_sweep_c:b2Vec2 = b2.m_sweep.c;
			var b2_sweep_a:Number = b2.m_sweep.a;
			
			var invMass1:Number = b1.m_mass * b1.m_invMass;
			var invI1:Number = b1.m_mass * b1.m_invI;
			var invMass2:Number = b2.m_mass * b2.m_invMass;
			var invI2:Number = b2.m_mass * b2.m_invI;
			//var normal:b2Vec2 = new b2Vec2(c.normal.x, c.normal.y);
			var normalX:Number = c.normal.x;
			var normalY:Number = c.normal.y;
			
			// Solver normal constraints
			var tCount:int = c.pointCount;
			for (var j:int = 0; j < tCount; ++j)
			{
				var ccp:b2ContactConstraintPoint = c.points[ j ];
				
				//r1 = b2Mul(b1->m_xf.R, ccp->localAnchor1 - b1->GetLocalCenter());
				tMat = b1.m_xf.R;
				tVec = b1.m_sweep.localCenter;
				var r1X:Number = ccp.localAnchor1.x - tVec.x;
				var r1Y:Number = ccp.localAnchor1.y - tVec.y;
				tX =  (tMat.col1.x * r1X + tMat.col2.x * r1Y);
				r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
				r1X = tX;
				
				//r2 = b2Mul(b2->m_xf.R, ccp->localAnchor2 - b2->GetLocalCenter());
				tMat = b2.m_xf.R;
				tVec = b2.m_sweep.localCenter;
				var r2X:Number = ccp.localAnchor2.x - tVec.x;
				var r2Y:Number = ccp.localAnchor2.y - tVec.y;
				var tX:Number =  (tMat.col1.x * r2X + tMat.col2.x * r2Y);
				r2Y = 			 (tMat.col1.y * r2X + tMat.col2.y * r2Y);
				r2X = tX;
				
				//b2Vec2 p1 = b1->m_sweep.c + r1;
				var p1X:Number = b1_sweep_c.x + r1X;
				var p1Y:Number = b1_sweep_c.y + r1Y;
				
				//b2Vec2 p2 = b2->m_sweep.c + r2;
				var p2X:Number = b2_sweep_c.x + r2X;
				var p2Y:Number = b2_sweep_c.y + r2Y;
				
				//var dp:b2Vec2 = b2Math.SubtractVV(p2, p1);
				var dpX:Number = p2X - p1X;
				var dpY:Number = p2Y - p1Y;
				
				// Approximate the current separation.
				//var separation:Number = b2Math.b2Dot(dp, normal) + ccp.separation;
				var separation:Number = (dpX*normalX + dpY*normalY) + ccp.separation;
				
				// Track max constraint error.
				minSeparation = b2Math.b2Min(minSeparation, separation);
				
				// Prevent large corrections and allow slop.
				var C:Number = baumgarte * b2Math.b2Clamp(separation + b2Settings.b2_linearSlop, -b2Settings.b2_maxLinearCorrection, 0.0);
				
				// Compute normal impulse
				var dImpulse:Number = -ccp.equalizedMass * C;
				
				//var P:b2Vec2 = b2Math.MulFV( dImpulse, normal );
				var PX:Number = dImpulse * normalX;
				var PY:Number = dImpulse * normalY;
				
				//b1.m_position.Subtract( b2Math.MulFV( invMass1, impulse ) );
				b1_sweep_c.x -= invMass1 * PX;
				b1_sweep_c.y -= invMass1 * PY;
				b1_sweep_a -= invI1 * (r1X * PY - r1Y * PX);//b2Math.b2CrossVV(r1, P);
				b1.m_sweep.a = b1_sweep_a;
				b1.SynchronizeTransform();
				
				//b2.m_position.Add( b2Math.MulFV( invMass2, P ) );
				b2_sweep_c.x += invMass2 * PX;
				b2_sweep_c.y += invMass2 * PY;
				b2_sweep_a += invI2 * (r2X * PY - r2Y * PX);//b2Math.b2CrossVV(r2, P);
				b2.m_sweep.a = b2_sweep_a;
				b2.SynchronizeTransform();
			}
			// Update body rotations
			//b1.m_sweep.a = b1_sweep_a;
			//b2.m_sweep.a = b2_sweep_a;
		}
		
		// We can't expect minSpeparation >= -b2_linearSlop because we don't
		// push the separation above -b2_linearSlop.
		return minSeparation >= -1.5 * b2Settings.b2_linearSlop;
	}
//#else
// Block solver
// TODO: Port block solver if it is ever enabled
//#endif
	private var m_step:b2TimeStep = new b2TimeStep();
	private var m_allocator:*;
	b2internal var m_constraints:Array = new Array();
	private var m_constraintCount:int;
};

}
