﻿/*
* Copyright (c) 2010 Adam Newgas http://www.boristhebrave.com
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

package Box2D.Dynamics.Controllers 
{
	import Box2D.Collision.b2AABB;
	import Box2D.Dynamics.b2Body;
	import Box2D.Dynamics.b2Fixture;
	import Box2D.Dynamics.b2World;
	
	/**
	 * Lists bodies that loosely fall inside a given AABB. 
	 * This corresponds to b2World#QueryAABB
	 */
	public class b2AABBBodyIterable implements IBodyIterable
	{
		
		public function b2AABBBodyIterable(world:b2World, aabb:b2AABB) 
		{
			m_world = world;
			m_aabb = aabb.Copy();
		}
		
		/** @inheritDoc */
		public function GetIterator():IBodyIterator
		{
			var iterator:b2ManualBodyIterator = new b2ManualBodyIterator();
			iterator.bodyList = new Vector.<b2Body>();
			return ResetIterator(iterator);
		}
		
		/** @inheritDoc */
		public function ResetIterator(iterator:IBodyIterator):IBodyIterator
		{
			var iterator2:b2ManualBodyIterator = iterator as b2ManualBodyIterator;
			var bodyList:Vector.<b2Body> = iterator2.bodyList;
			bodyList.length = 0;
			function callback(fixture:b2Fixture):Boolean
			{
				var body:b2Body = fixture.GetBody();
				if(bodyList.indexOf(body)===-1)
					bodyList[bodyList.length] = body;
				return true;
			}
			m_world.QueryAABB(callback, m_aabb);
			return iterator2;
		}
		
		private var m_world:b2World;
		private var m_aabb:b2AABB;
	}
	
}