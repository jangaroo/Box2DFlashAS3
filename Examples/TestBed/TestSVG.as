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
	import com.boristhebrave.Box2DWith.b2SVG;
	import com.kerchmar.tim.bayazit.PolyDecompBayazit;
	import flash.geom.Point;
	
	public class TestSVG extends Test {
		
		public var m_plusBody:b2Body;
		
		public function TestSVG(){
			
			// Set Text field
			Main.m_aboutText.text = "Edge + SVG + Convex decomposition";
			
			// Thanks w3schools.com for this spiral curve
			// C commands do a cubic bezier, which the parser will split
			// up for you into short segments
			var svg:XML = 
				<svg width="100%" height="100%" version="1.1"
				xmlns="http://www.w3.org/2000/svg">

				<path d="M153 134 \
				C153 134 151 134 151 134 \
				C151 139 153 144 156 144 \
				C164 144 171 139 171 134 \
				C171 122 164 114 156 114 \
				C142 114 131 122 131 134 \
				C131 150 142 164 156 164 \
				C175 164 191 150 191 134 \
				C191 111 175  94 156  94 \
				C131  94 111 111 111 134 \
				C111 161 131 184 156 184 \
				C186 184 211 161 211 134 \
				C211 100 186  74 156  74"
				/>
				</svg>;
				
			// This converts SVG to a series of polylines.
			var paths:Array/*Vector.<b2Vec2>*/ = b2SVG.parseSVG(svg, m_physScale, 10);
			
			// Write out the polylines using thin edges
			writeLines(paths);
			
			// Make a ball that will go around our spiral
			var bd:b2BodyDef = new b2BodyDef();
			bd.type = b2Body.b2_dynamicBody;
			bd.position.x = 100 / m_physScale;
			bd.position.y = 80 / m_physScale;
			bd.linearVelocity.x = 15;
			var body:b2Body = m_world.CreateBody(bd);
			body.CreateFixture2(new b2CircleShape(.2), 1.0);
			
			// Another SVG, this time describing a plus block
			// h and v are horizontal and vertical lines.
			svg = 
				<svg width="100%" height="100%" version="1.1"
				xmlns="http://www.w3.org/2000/svg">

				<path d = "M -45 -45 h30v-30h30v30h30v30h-30v30h-30v-30h-30z"/>
				</svg>;
			
			// Again, parse it
			paths = b2SVG.parseSVG(svg, m_physScale, 4);
			
			// This time, use convex decomposition to convert into convex polygons
			// Because we are making a solid shape, there's no problems with it being dynamic.
			bd = new b2BodyDef();
			bd.type = b2Body.b2_dynamicBody;
			bd.position.x = 400 / m_physScale;
			bd.position.y = 200 / m_physScale;
			m_plusBody = m_world.CreateBody(bd);
			writePolys(paths);
		}
		
		private function writeLines(paths:Array/*Vector.<b2Vec2>*/):void
		{
			for each(var path:Array/*b2Vec2*/ in paths)
			{
				var last:b2Vec2;
				for each(var v:b2Vec2 in path)
				{
					if (last)
					{
						var s:b2Shape = b2PolygonShape.AsEdge(last, v);
						m_world.GetGroundBody().CreateFixture2(s, 0.0);
					}
					last = v;
				}
			}
		}
		
		private function writePolys(paths:Array/*Vector.<b2Vec2>*/):void
		{
			for each(var path:Array/*b2Vec2*/ in paths)
			{
				var points:Array = [];
				for each(var v:b2Vec2 in path)
				{
					points.push(new Point(v.x, v.y));
				}
				var decomp:PolyDecompBayazit = new PolyDecompBayazit(points);
				decomp.decompose(decomposeCallback);
			}
		}
		
		private function decomposeCallback(decomp:PolyDecompBayazit):void
		{
			// Make a polygon out of the points
			var points:Array/*b2Vec2*/ = new Array/*b2Vec2*/();
			for each(var p:Point in decomp.points)
			{
				points.push(new b2Vec2(p.x, p.y));
			}
			
			var s:b2Shape = b2PolygonShape.AsVector(points);
			
			m_plusBody.CreateFixture2(s, 0.1);
		}
		
		
		//======================
		// Member Data 
		//======================
	}
	
}