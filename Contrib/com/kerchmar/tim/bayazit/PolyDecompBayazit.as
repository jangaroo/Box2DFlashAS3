/*
* Copyright (c) 2006-2007 Tim Kerchmar http://ptymn.blogspot.com/
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

package com.kerchmar.tim.bayazit {	
	import flash.geom.*;
	
	public class PolyDecompBayazit {
		public static function area(a:Point, b:Point, c:Point):Number {
			return (((b.x - a.x)*(c.y - a.y))-((c.x - a.x)*(b.y - a.y)));
		}
		
		public static function right(a:Point, b:Point, c:Point):Boolean {
			return area(a, b, c) < 0;
		}

		public static function rightOn(a:Point, b:Point, c:Point):Boolean {
			return area(a, b, c) <= 0;
		}
		
		public static function left(a:Point, b:Point, c:Point):Boolean {
		    return area(a, b, c) > 0;
		}
		
		public static function leftOn(a:Point, b:Point, c:Point):Boolean {
		    return area(a, b, c) >= 0;
		}
		
		public static function sqdist(a:Point, b:Point):Number {
			var dx:Number = b.x - a.x;
			var dy:Number = b.y - a.y;
			return dx * dx + dy * dy;
		}
		
		public static function getIntersection(start1:Point, end1:Point, start2:Point, end2:Point):Point {
			var a1:Number = end1.y - start1.y;
			var b1:Number = start1.x - end1.x;
			var c1:Number = a1 * start1.x + b1 * start1.y;
			var a2:Number = end2.y - start2.y;
			var b2:Number = start2.x - end2.x;
			var c2:Number = a2 * start2.x + b2 * start2.y;
			var det:Number = a1 * b2 - a2*b1;
			
			if (Math.abs(det) > Number.MIN_VALUE) { // lines are not parallel
				return new Point((b2 * c1 - b1 * c2) / det,  (a1 * c2 - a2 * c1) / det);
			}
			return null;
		}
		
		public function combineColinearPoints():void {
			// combine similar points
			var combinedPoints:Array = [];
			
			for(var i:int = 0; i < points.length; i++) {
				var a:Point = at(i - 1), b:Point = at(i), c:Point = at(i + 1);
				
				if(getIntersection(a, b, b, c) != null)
					combinedPoints.push(b);
			}
			
			points = combinedPoints;
		}

		public var points:Array;
		
		public function PolyDecompBayazit(points:Array) {
			this.points = points;
			
			combineClosePoints();
			combineColinearPoints();
			makeCCW();
		}
		
		public function combineClosePoints():void {
			var combinedPoints:Array = [];
			
			for(var i:int = 0; i < points.length; i++) {
				var a:Point = at(i);
				var b:Point = at(i + 1);

				if(sqdist(a, b) > Number.MIN_VALUE)
					combinedPoints.push(a);
			}
			
			points = combinedPoints;
		}
		
		public function at(i:int):Point {
			var s:int = points.length;
			return points[(i + s) % s];
		}

		public function isReflex(i:int):Boolean {
		    return right(at(i - 1), at(i), at(i + 1));
		}
		
		public function polyFromRange(lower:int, upper:int):PolyDecompBayazit {
			if(lower < upper)
				return new PolyDecompBayazit(points.slice(lower, upper + 1));
			else
				return new PolyDecompBayazit(points.slice(lower).concat(points.slice(0, upper + 1)));
		}
		
		public function decompose(callback:Function):void {
			if(points.length < 3) return;
			
			for (var i:int = 0; i < points.length; ++i) {
				if (isReflex(i)) {
					// Find closest two vertices in range from a reflex point (two the vertices are by going CW and CCW around polygon)
					// See first diagram on this page: http://mnbayazit.com/406/bayazit
					var upperDist:Number = Number.MAX_VALUE, upperIntersection:Point, upperIndex:int;
					var lowerDist:Number = Number.MAX_VALUE, lowerIntersection:Point, lowerIndex:int;
					
					for (var j:int = 0; j < points.length; ++j) {
						if (left(at(i - 1), at(i), at(j)) && rightOn(at(i - 1), at(i), at(j - 1))) { // if line intersects with an edge
							var intersectionPoint:Point = getIntersection(at(i - 1), at(i), at(j), at(j - 1)); // find the point of intersection
							if (right(at(i + 1), at(i), intersectionPoint)) { // make sure it's inside the poly
								var distance:Number = sqdist(at(i), intersectionPoint);
								if (distance < lowerDist) { // keep only the closest intersection
									lowerDist = distance;
									lowerIntersection = intersectionPoint;
									lowerIndex = j;
								}
							}
						}
						if (left(at(i + 1), at(i), at(j + 1)) && rightOn(at(i + 1), at(i), at(j))) {
							intersectionPoint = getIntersection(at(i + 1), at(i), at(j), at(j + 1));
							if (left(at(i - 1), at(i), intersectionPoint)) {
								distance = sqdist(at(i), intersectionPoint);
								if (distance < upperDist) {
									upperDist = distance;
									upperIntersection = intersectionPoint;
									upperIndex = j;
								}
							}
						}
					}
					
					var lowerPoly:PolyDecompBayazit, upperPoly:PolyDecompBayazit;
		
					// if there are no vertices to connect to, choose a point in the middle
					if (lowerIndex == (upperIndex + 1) % points.length) {
						var steinerPoint:Point = new Point(
							(lowerIntersection.x + upperIntersection.x) * 0.5,
							(lowerIntersection.y + upperIntersection.y) * 0.5);
		
						lowerPoly = polyFromRange(i, upperIndex);
						lowerPoly.points.push(steinerPoint);
		
						if (i < upperIndex)
							upperPoly = polyFromRange(lowerIndex, i);
						else
							upperPoly = polyFromRange(0, i);
						upperPoly.points.push(steinerPoint);
					} else {
						// connect to the closest point within the triangle
		
						// at(n) handles mod points.length, so increase upperIndex to make for loop easy
						if (lowerIndex > upperIndex) upperIndex += points.length;
						
						// Find closest point in range
						var closestIndex:int, closestDist:Number = Number.MAX_VALUE, closestVert:Point;
						for (j = lowerIndex; j <= upperIndex; ++j) {
							if (leftOn(at(i - 1), at(i), at(j)) && rightOn(at(i + 1), at(i), at(j))) {
								distance = sqdist(at(i), at(j));
								if (distance < closestDist) {
									closestDist = distance;
									closestVert = at(j);
									closestIndex = j % points.length;
								}
							}
						}
		
						lowerPoly = polyFromRange(i, closestIndex);
						upperPoly = polyFromRange(closestIndex, i);
					}
		
					// solve smallest poly first
					if (lowerPoly.points.length < upperPoly.points.length) {
						lowerPoly.decompose(callback);
						upperPoly.decompose(callback);
					} else {
						upperPoly.decompose(callback);
						lowerPoly.decompose(callback);
					}
					return;
				}
			}
			
			if(points.length >= 3) callback(this);
		}
		
		public function makeCCW():void {
			var br:int = 0;
		
			// find bottom right point
			for (var i:int = 1; i < points.length; ++i) {
				if (at(i).y < at(br).y || (at(i).y == at(br).y && at(i).x > at(br).x)) {
					br = i;
				}
			}
		
			// reverse poly if clockwise
			if (!left(at(br - 1), at(br), at(br + 1))) {
				points.reverse();
			}
		}
	}
}
