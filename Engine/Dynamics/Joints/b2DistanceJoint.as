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

package Engine.Dynamics.Joints{
	
import Engine.Common.Math.*
import Engine.Common.*
import Engine.Dynamics.*;

// C = norm(p2 - p1) - L
// u = (p2 - p1) / norm(p2 - p1)
// Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
// J = [-u -cross(r1, u) u cross(r2, u)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2

public class b2DistanceJoint extends b2Joint
{
	//--------------- Internals Below -------------------

	public function b2DistanceJoint(def:b2DistanceJointDef){
		super(def);
		
		m_localAnchor1 = b2Math.b2MulTMV(m_body1.m_R, b2Math.SubtractVV(def.anchorPoint1, m_body1.m_position));
		m_localAnchor2 = b2Math.b2MulTMV(m_body2.m_R, b2Math.SubtractVV(def.anchorPoint2, m_body2.m_position));
		
		var d:b2Vec2 = b2Math.SubtractVV( def.anchorPoint2, def.anchorPoint1 );
		m_length = d.Length();
		m_impulse = 0.0;
	}

	public override function PrepareVelocitySolver(){
		// Compute the effective mass matrix.
		var r1:b2Vec2 = b2Math.b2MulMV(m_body1.m_R, m_localAnchor1);
		var r2:b2Vec2 = b2Math.b2MulMV(m_body2.m_R, m_localAnchor2);
		m_u = b2Math.SubtractVV( b2Math.SubtractVV( b2Math.AddVV( m_body2.m_position, r2), m_body1.m_position ), r1);
		
		// Handle singularity.
		var length:Number = m_u.Length();
		if (length > b2Settings.b2_linearSlop)
		{
			m_u.Multiply( 1.0 / length );
		}
		else
		{
			m_u.Set(0.0, 0.0);
		}
		
		var cr1u:Number = b2Math.b2CrossVV(r1, m_u);
		var cr2u:Number = b2Math.b2CrossVV(r2, m_u);
		m_mass = m_body1.m_invMass + m_body1.m_invI * cr1u * cr1u + m_body2.m_invMass + m_body2.m_invI * cr2u * cr2u;
		//b2Settings.b2Assert(m_mass > Number.MIN_VALUE);
		m_mass = 1.0 / m_mass;
		
		// Warm starting.
		if (b2World.s_enableWarmStarting)
		{
			var P:b2Vec2 = b2Math.MulFV(m_impulse , m_u);
			m_body1.m_linearVelocity.Subtract( b2Math.MulFV(m_body1.m_invMass , P));
			m_body1.m_angularVelocity -= m_body1.m_invI * b2Math.b2CrossVV(r1, P);
			m_body2.m_linearVelocity.Add( b2Math.MulFV (m_body2.m_invMass , P ) );
			m_body2.m_angularVelocity += m_body2.m_invI * b2Math.b2CrossVV(r2, P);
		}
		else{
			m_impulse = 0.0;
		}
	}
	
	public override function SolveVelocityConstraints(step:b2StepInfo){
		
		var r1:b2Vec2 = b2Math.b2MulMV(m_body1.m_R, m_localAnchor1);
		var r2:b2Vec2 = b2Math.b2MulMV(m_body2.m_R, m_localAnchor2);
		
		// Cdot = dot(u, v + cross(w, r))
		var v1:b2Vec2 = b2Math.AddVV(m_body1.m_linearVelocity , b2Math.b2CrossFV(m_body1.m_angularVelocity, r1));
		var v2:b2Vec2 = b2Math.AddVV(m_body2.m_linearVelocity , b2Math.b2CrossFV(m_body2.m_angularVelocity, r2));
		var Cdot:Number = b2Math.b2Dot(m_u, b2Math.SubtractVV(v2 , v1));
		var impulse:Number = -m_mass * Cdot;
		m_impulse += impulse;
		
		var P:b2Vec2 = b2Math.MulFV(impulse , m_u);
		m_body1.m_linearVelocity.Subtract( b2Math.MulFV(m_body1.m_invMass , P ));
		m_body1.m_angularVelocity -= m_body1.m_invI * b2Math.b2CrossVV(r1, P);
		m_body2.m_linearVelocity.Add( b2Math.MulFV( m_body2.m_invMass , P ));
		m_body2.m_angularVelocity += m_body2.m_invI * b2Math.b2CrossVV(r2, P);
	}
	
	public override function SolvePositionConstraints():Boolean{
		var r1:b2Vec2 = b2Math.b2MulMV(m_body1.m_R, m_localAnchor1);
		var r2:b2Vec2 = b2Math.b2MulMV(m_body2.m_R, m_localAnchor2);
		var d:b2Vec2 = b2Math.SubtractVV( b2Math.SubtractVV( b2Math.AddVV(m_body2.m_position , r2) , m_body1.m_position) , r1);
		
		var length:Number = d.Normalize();
		var C:Number = length - m_length;
		C = b2Math.b2Clamp(C, -b2Settings.b2_maxLinearCorrection, b2Settings.b2_maxLinearCorrection);
		
		var impulse:Number = -m_mass * C;
		m_u.SetV(d);
		var P:b2Vec2 = b2Math.MulFV(impulse , m_u);
		
		m_body1.m_position.Subtract( b2Math.MulFV(m_body1.m_invMass , P));
		m_body1.m_rotation -= m_body1.m_invI * b2Math.b2CrossVV(r1, P);
		m_body2.m_position.Add( b2Math.MulFV(m_body2.m_invMass , P));
		m_body2.m_rotation += m_body2.m_invI * b2Math.b2CrossVV(r2, P);
		
		m_body1.m_R.Set(m_body1.m_rotation);
		m_body2.m_R.Set(m_body2.m_rotation);
		
		return b2Math.b2Abs(C) < b2Settings.b2_linearSlop;
	}
	
	public override function GetAnchor1():b2Vec2{
		return b2Math.AddVV(m_body1.m_position , b2Math.b2MulMV(m_body1.m_R, m_localAnchor1));
	}
	public override function GetAnchor2():b2Vec2{
		return b2Math.AddVV(m_body2.m_position , b2Math.b2MulMV(m_body2.m_R, m_localAnchor2));
	}
	
	public override function GetReactionForce(invTimeStep:Number):b2Vec2
	{
		//var F:b2Vec2 = (m_impulse * invTimeStep) * m_u;
		var F:b2Vec2 = new b2Vec2();
		F.SetV(m_u);
		F.Multiply(m_impulse * invTimeStep);
		return F;
	}

	public override function GetReactionTorque(invTimeStep:Number):Number
	{
		//NOT_USED(invTimeStep);
		return 0.0;
	}

	public var m_localAnchor1:b2Vec2;
	public var m_localAnchor2:b2Vec2;
	public var m_u:b2Vec2;
	public var m_impulse:Number;
	public var m_mass:Number;	// effective mass for the constraint.
	public var m_length:Number;
};

}
