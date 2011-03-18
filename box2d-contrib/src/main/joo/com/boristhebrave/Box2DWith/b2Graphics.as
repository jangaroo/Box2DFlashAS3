/*
* Copyright (c) 2009 Adam Newgas http://www.boristhebrave.com
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

package com.boristhebrave.Box2DWith 
{
	import Box2D.Collision.Shapes.*;
	import Box2D.Common.Math.*;
	import Box2D.Dynamics.*;
	import Box2D.Dynamics.Joints.*;
	
	import flash.display.*;
	/**
	 * A re-implementation of debug drawing without the hassles of Box2D's API.
	 * By drawing into Sprites which you persist through frames, the methods here
	 * are appropriate for production release.
	 */
	public class b2Graphics 
	{
		/** The alpha to use for filling in bodies. */
		public var fillAlpha:Number = 0.5;
		/** The color to use for static bodies. */
		public var staticColor:uint = 0x80E680;
		/** The color to use for kinematic bodies. */
		public var kinematicColor:uint = 0x8080E6;
		/** The color to use for sleeping dynamic bodies. */
		public var sleepColor:uint = 0x8080E6;
		/** The color to use for awake dynamic bodies. */
		public var awakeColor:uint = 0xE6E6E6;
		/** The color to use for drawing joints. */
		public var jointColor:uint = 0x80CDCD;
		/** The line thickness to use for drawing, in pixels. */
		public var lineThickness:Number = 1;
		/** The size to draw xforms at, in units. */
		public var xformScale:Number = 3;
		
		/** The wrapped Graphics to draw to. */
		public var graphics:Graphics;
		/** The scale between pixels and Box2D units, in pixels/unit. */
		public var physScale:Number;
		
		/**
		 * Creates a wrapper for Graphics, with it's own set of drawing properties.
		 * @param	graphics	The Graphics object to wrap.
		 * @param	physScale	The scaling to use, in pixels/unit.
		 */
		public function b2Graphics(graphics:Graphics, physScale:Number):void
		{
			this.graphics = graphics;
			this.physScale = physScale;
		}
		
		/**
		 * Draws an unstyled line.
		 */
		public function drawSegment(p1:b2Vec2, p2:b2Vec2):void
		{
			graphics.moveTo(p1.x*physScale, p1.y*physScale);
			graphics.lineTo(p2.x*physScale, p2.y*physScale);
		}
		
		/**
		 * Draws the given shape in local co-ordinates, without setting any style properties.
		 * Use this instead of drawShape if you want a more customized looking shape.
		 */
		public function drawShapeOutline(shape:b2Shape):void
		{
			if (shape is b2CircleShape)
			{
				var circle:b2CircleShape = shape as b2CircleShape;
				var localPosition:b2Vec2 = circle.GetLocalPosition();
				graphics.drawCircle(localPosition.x* physScale, localPosition.y* physScale, circle.GetRadius() * physScale);
				return;
			}
			if (shape is b2PolygonShape)
			{
				var vertices:Vector.<b2Vec2> = (shape as b2PolygonShape).GetVertices();
				var n:int = (shape as b2PolygonShape).GetVertexCount();
				if (!n)
					return;
				graphics.moveTo(vertices[n-1].x* physScale, vertices[n-1].y* physScale);
				for (var i:int = 0; i < n; i++)
				{
					graphics.lineTo(vertices[i].x* physScale, vertices[i].y* physScale);
				}
				return;
			}
			throw "Unrecognized shape: " + shape;
		}
		
		/**
		 * Draws the given shape in world co-ordinates, without setting any style properties.
		 * Use this instead of drawShape if you want a more customized looking shape.
		 */
		public function drawFixtureOutline(fixture:b2Fixture):void
		{
			var xf:b2Transform = fixture.GetBody().GetTransform();
			var shape:b2Shape = fixture.GetShape();
			if (shape is b2CircleShape)
			{
				var circle:b2CircleShape = shape as b2CircleShape;
				var localPosition:b2Vec2 = b2Math.MulX(xf,circle.GetLocalPosition());
				graphics.drawCircle(localPosition.x* physScale, localPosition.y* physScale, circle.GetRadius() * physScale);
				return;
			}
			if (shape is b2PolygonShape)
			{
				var vertices:Vector.<b2Vec2> = (shape as b2PolygonShape).GetVertices();
				vertices = vertices.map(function(v:b2Vec2, a:*, b:*):b2Vec2 { return b2Math.MulX(xf, v);} );
				var n:int = (shape as b2PolygonShape).GetVertexCount();
				if (!n)
					return;
				graphics.moveTo(vertices[n-1].x* physScale, vertices[n-1].y* physScale);
				for (var i:int = 0; i < n; i++)
				{
					graphics.lineTo(vertices[i].x* physScale, vertices[i].y* physScale);
				}
				return;
			}
			throw "Unrecognized shape: " + shape;
		}

		/**
		 * Draws the given body in local cordinates, without setting any style properties.
		 * Use this instead of drawBodyLocal if you want a more customized looking shape.
		 */
		public function drawBodyOutlineLocal(body:b2Body):void
		{
			for (var f:b2Fixture = body.GetFixtureList(); f; f = f.GetNext())
			{
				drawShapeOutline(f.GetShape());
			}
		}
		
		/**
		 * Draws the given body, without setting any style properties.
		 * Use this instead of drawBody if you want a more customized looking shape.
		 */
		public function drawBodyOutline(body:b2Body):void
		{
			for (var f:b2Fixture = body.GetFixtureList(); f; f = f.GetNext())
			{
				drawFixtureOutline(f);
			}
		}
		
		public function computeBodyColor(body:b2Body):int
		{
			switch(body.GetType())
			{
				case b2Body.b2_staticBody:
					return staticColor;
				case b2Body.b2_kinematicBody:
					return kinematicColor;
				case b2Body.b2_dynamicBody:
					if(!body.IsAwake())
					{
						return sleepColor;
					}
					else
					{
						return awakeColor;
					}
			}
			return 0;
		}
		
		/**
		 * Draws a body in local co-ordinates, including styling.
		 */
		public function drawBodyLocal(body:b2Body):void
		{
			var color:uint = computeBodyColor(body);
			graphics.lineStyle(1, color);
			graphics.beginFill(color,fillAlpha);
			drawBodyOutlineLocal(body);
			graphics.endFill();
		}
		
		/**
		 * Draws a body, including styling.
		 */
		public function drawBody(body:b2Body):void
		{
			var color:uint = computeBodyColor(body);
			graphics.lineStyle(1, color);
			graphics.beginFill(color,fillAlpha);
			drawBodyOutline(body);
			graphics.endFill();
		}
		
		/**
		 * Draws the given joint, without setting any style properties.
		 * Use this instead of drawJoint if you want a more customized looking shape.
		 */
		public function drawJointOutline(joint:b2Joint):void
		{
			var b1:b2Body = joint.GetBodyA();
			var b2:b2Body = joint.GetBodyB();
			//var xf1:b2XForm = b1.GetXForm();
			//var xf2:b2XForm = b2.GetXForm();
			var x1:b2Vec2 = b1.GetPosition();
			var x2:b2Vec2 = b2.GetPosition();
			var p1:b2Vec2 = joint.GetAnchorA();
			var p2:b2Vec2 = joint.GetAnchorB();
			if (joint is b2DistanceJoint || joint is b2MouseJoint)
			{
				drawSegment(p1, p2);
				return;
			}
			if (joint is b2PulleyJoint)
			{
				var pulley:b2PulleyJoint = (joint as b2PulleyJoint);
				var s1:b2Vec2 = pulley.GetGroundAnchorA();
				var s2:b2Vec2 = pulley.GetGroundAnchorB();
				drawSegment(s1, p1);
				drawSegment(s2, p2);
				drawSegment(s1, s2);
				return;
			}
			//Default
			var groundBody:b2Body = b1.GetWorld().GetGroundBody();
			if (b1 != groundBody)
				drawSegment(x1, p1);
			drawSegment(p1, p2);
			if (b2 != groundBody)
				drawSegment(p2, x2);
		}
		
		/**
		 * Draws a joint, including styling.
		 */
		public function drawJoint(joint:b2Joint):void
		{
			graphics.lineStyle(lineThickness, jointColor);
			drawJointOutline(joint);
		}
		
		/**
		 * Draws a world, including styling. Note that it is more efficient to draw bodies into
		 * separate DisplayObjects once, and then move them around, than to call drawWorld once
		 * per frame.
		 * @deprecated
		 */
		public function drawWorld(world:b2World):void
		{
			for (var body:b2Body = world.GetBodyList(); body; body = body.GetNext())
			{
				drawBody(body);
			}
			for (var joint:b2Joint = world.GetJointList(); joint; joint = joint.GetNext())
			{
				drawJoint(joint);
			}
		}
	}
	
}