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

package Box2D.Collision{
	
import Box2D.Collision.*;
import Box2D.Common.Math.*;

import Box2D.Common.b2internal;
use namespace b2internal;

/**
* An axis aligned bounding box.
*/
public class b2AABB
{
	/**
	* Verify that the bounds are sorted.
	*/
	public function IsValid():Boolean{
		//b2Vec2 d = upperBound - lowerBound;;
		var dX:Number = upperBound.x - lowerBound.x;
		var dY:Number = upperBound.y - lowerBound.y;
		var valid:Boolean = dX >= 0.0 && dY >= 0.0;
		valid = valid && lowerBound.IsValid() && upperBound.IsValid();
		return valid;
	}
	
	/** Get the center of the AABB. */
	public function GetCenter():b2Vec2
	{
		return new b2Vec2( (lowerBound.x + upperBound.x) / 2,
		                   (lowerBound.y + upperBound.y) / 2);
	}
	
	/** Get the extents of the AABB (half-widths). */
	public function GetExtents():b2Vec2
	{
		return new b2Vec2( (upperBound.x - lowerBound.x) / 2,
		                   (upperBound.y - lowerBound.y) / 2);
	}
	
	public function Contains(aabb:b2AABB):Boolean
	{
		var result:Boolean = true;
		result &&= lowerBound.x <= aabb.lowerBound.x;
		result &&= lowerBound.y <= aabb.lowerBound.y;
		result &&= aabb.upperBound.x <= upperBound.x;
		result &&= aabb.upperBound.y <= upperBound.y;
		return result;
	}
	
	
	/** Combine two AABBs into one. */
	public static function Combine(aabb1:b2AABB, aabb2:b2AABB):b2AABB
	{
		var aabb:b2AABB = new b2AABB();
		aabb.lowerBound.x = Math.min(aabb1.lowerBound.x, aabb2.lowerBound.x);
		aabb.lowerBound.y = Math.min(aabb1.lowerBound.y, aabb2.lowerBound.y);
		aabb.upperBound.x = Math.max(aabb1.upperBound.x, aabb2.upperBound.x);
		aabb.upperBound.y = Math.max(aabb1.upperBound.y, aabb2.upperBound.y);
		return aabb;
	}

	/** The lower vertex */
	public var lowerBound:b2Vec2 = new b2Vec2();
	/** The upper vertex */
	public var upperBound:b2Vec2 = new b2Vec2();
};


}