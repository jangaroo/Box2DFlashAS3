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


package TestBed{
	
	
	
	import Box2D.Dynamics.*;
	import Box2D.Collision.*;
	import Box2D.Collision.Shapes.*;
	import Box2D.Dynamics.Joints.*;
	import Box2D.Dynamics.Contacts.*;
	import Box2D.Common.*;
	import Box2D.Common.Math.*;
	
	
	
	public class TestRaycast extends Test{
		
		private var laser:b2Body;
		
		public function TestRaycast(){
			
			// Set Text field
			Main.m_aboutText.text = "Raycast";
			
			m_world.SetGravity(new b2Vec2(0,0));
			
			var ground:b2Body = m_world.GetGroundBody();
			
			var box:b2PolygonDef = new b2PolygonDef();
			box.SetAsBox(30 / m_physScale, 4 / m_physScale);
			box.density = 4;
			box.friction = 0.4;
			box.restitution = 0.3;
			box.userData="laser";
			var bd:b2BodyDef = new b2BodyDef();
			bd.position.Set(320 / m_physScale, 150 / m_physScale);
			laser = m_world.CreateBody(bd);
			laser.CreateShape(box);
			laser.SetMassFromShapes();
			laser.SetXForm(laser.GetPosition(),0.5);
			
			var cd:b2CircleDef = new b2CircleDef();
			cd.radius = 30 / m_physScale;
			cd.density = 4;
			cd.friction = 0.4;
			cd.restitution = 0.3;
			cd.userData="circle";
			bd.position.Set(100 / m_physScale, 100 / m_physScale);
			var body:b2Body = m_world.CreateBody(bd);
			body.CreateShape(cd);
			body.SetMassFromShapes();
		}
		
		
		//======================
		// Member Data 
		//======================
		
		public override function Update():void{
			super.Update();
			
			var segment:b2Segment = new b2Segment();
			segment.p1.SetV(laser.GetWorldPoint(new b2Vec2(30.1/m_physScale,0)))
			segment.p2.SetV(laser.GetWorldVector(new b2Vec2(100/m_physScale,0)))
			segment.p2.Add(segment.p1);
			
			//Example call for b2World.Raycast
			//var shapes:Array=new Array();
			//var n:Number=1;
			//n = m_world.m_broadPhase.QuerySegment(segment,shapes,n,null);
			//n = m_world.Raycast(segment,shapes,n,null);
			
			var lambdaRef:Array=[1];
			var normal:b2Vec2=new b2Vec2();
			var shape:b2Shape = m_world.RaycastOne(segment,lambdaRef,normal,true,null);
			Main.m_aboutText.text=""
			if(shape)
			{
				if(shape.GetUserData())
					Main.m_aboutText.text = shape.GetUserData();
			}
			else
			{
				lambdaRef=[1];
			}
			
			
			var lambda:Number = lambdaRef[0];
			m_sprite.graphics.lineStyle(1,0xff0000,1);
			m_sprite.graphics.moveTo(segment.p1.x * m_physScale, segment.p1.y * m_physScale);
			m_sprite.graphics.lineTo( 	(segment.p2.x * lambda + (1-lambda) * segment.p1.x) * m_physScale,
										(segment.p2.y * lambda + (1-lambda) * segment.p1.y) * m_physScale);
		}
	}
	
}