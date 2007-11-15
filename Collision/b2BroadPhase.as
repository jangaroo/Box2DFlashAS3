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

package Engine.Collision{
	
	
import Engine.Common.*
import Engine.Collision.*
import Engine.Common.Math.*
	

/*
This broad phase uses the Sweep and Prune algorithm as described in:
Collision Detection in Interactive 3D Environments by Gino van den Bergen
Also, some ideas, such as using integral values for fast compares comes from
Bullet (http:/www.bulletphysics.com).
*/


// Notes:
// - we use bound arrays instead of linked lists for cache coherence.
// - we use quantized integral values for fast compares.
// - we use short indices rather than pointers to save memory.
// - we use a stabbing count for fast overlap queries (less than order N).
// - we also use a time stamp on each proxy to speed up the registration of
//   overlap query results.
// - where possible, we compare bound indices instead of values to reduce
//   cache misses (TODO_ERIN).
// - no broadphase is perfect and neither is this one: it is not great for huge
//   worlds (use a multi-SAP instead), it is not great for large objects.

public class b2BroadPhase
{
//public:
	public function b2BroadPhase(worldAABB:b2AABB, callback:b2PairCallback){
		//b2Settings.b2Assert(worldAABB.IsValid());
		var i:int;
		
		m_worldAABB = worldAABB;
		m_pairCallback = callback;
		m_proxyCount = 0;
		
		// query results
		for (i = 0; i < b2Settings.b2_maxProxies; i++){
			m_queryResults[i] = 0;
		}
		
		// bounds array
		m_bounds = new Array(2);
		for (i = 0; i < 2; i++){
			m_bounds[i] = new Array(2*b2Settings.b2_maxProxies);
			for (var j:int = 0; j < 2*b2Settings.b2_maxProxies; j++){
				m_bounds[i][j] = new b2Bound();
			}
		}
		
		// pair buffer
		for (i = 0; i < b2Settings.b2_maxPairs; i++){
			m_pairBuffer[i] = new b2BufferedPair();
		}
		
		//var d:b2Vec2 = b2Math.SubtractVV(worldAABB.maxVertex, worldAABB.minVertex);
		var dX:Number = worldAABB.maxVertex.x;
		var dY:Number = worldAABB.maxVertex.y;
		dX -= worldAABB.minVertex.x;
		dY -= worldAABB.minVertex.y;
		
		m_quantizationFactor.x = b2Settings.USHRT_MAX / dX;
		m_quantizationFactor.y = b2Settings.USHRT_MAX / dY;
		
		var tProxy:b2Proxy;
		for (i = 0; i < b2Settings.b2_maxProxies - 1; ++i)
		{
			tProxy = new b2Proxy();
			m_proxyPool[i] = tProxy;
			tProxy.SetNext(i + 1);
			tProxy.timeStamp = 0;
			tProxy.overlapCount = b2_invalid;
			tProxy.userData = null;
		}
		tProxy = new b2Proxy();
		m_proxyPool[b2Settings.b2_maxProxies-1] = tProxy;
		tProxy.SetNext(b2Pair.b2_nullProxy);
		tProxy.timeStamp = 0;
		tProxy.overlapCount = b2_invalid;
		tProxy.userData = null;
		m_freeProxy = 0;
		
		m_pairBufferCount = 0;
		
		m_timeStamp = 1;
		m_queryResultCount = 0;
	}
	//~b2BroadPhase();
	
	// Use this to see if your proxy is in range. If it is not in range,
	// it should be destroyed. Otherwise you may get O(m^2) pairs, where m
	// is the number of proxies that are out of range.
	public function InRange(aabb:b2AABB):Boolean{
		//var d:b2Vec2 = b2Math.b2MaxV(b2Math.SubtractVV(aabb.minVertex, m_worldAABB.maxVertex), b2Math.SubtractVV(m_worldAABB.minVertex, aabb.maxVertex));
		var dX:Number;
		var dY:Number;
		var d2X:Number;
		var d2Y:Number;
		
		dX = aabb.minVertex.x;
		dY = aabb.minVertex.y;
		dX -= m_worldAABB.maxVertex.x;
		dY -= m_worldAABB.maxVertex.y;
		
		d2X = m_worldAABB.minVertex.x;
		d2Y = m_worldAABB.minVertex.y;
		d2X -= aabb.maxVertex.x;
		d2Y -= aabb.maxVertex.y;
		
		dX = b2Math.b2Max(dX, d2X);
		dY = b2Math.b2Max(dY, d2Y);
		
		return b2Math.b2Max(dX, dY) < 0.0;
	}
	
	// Get a single proxy. Returns NULL if the id is invalid.
	public function GetProxy(proxyId:int):b2Proxy{
		if (proxyId == b2Pair.b2_nullProxy || m_proxyPool[proxyId].IsValid() == false)
		{
			return null;
		}
		
		return m_proxyPool[ proxyId ];
	}

	// Create and destroy proxies. These call Flush first.
	public function CreateProxy(aabb:b2AABB, groupIndex:int, categoryBits:int, maskBits:int, userData:*):uint{
		var index:uint;
		var proxy:b2Proxy;
		
		//b2Settings.b2Assert(m_freeProxy != b2Pair.b2_nullProxy);
		if (m_freeProxy == b2Pair.b2_nullProxy)
		{
			//b2Settings.b2Assert(false);
			return b2Pair.b2_nullProxy;
		}
		
		// Flush the pair buffer
		Flush();
		
		var proxyId:uint = m_freeProxy;
		proxy = m_proxyPool[ proxyId ];
		m_freeProxy = proxy.GetNext();
		
		proxy.overlapCount = 0;
		proxy.groupIndex = groupIndex;
		proxy.categoryBits = categoryBits;
		proxy.maskBits = maskBits;
		proxy.userData = userData;
		
		//b2Settings.b2Assert(m_proxyCount < b2Settings.b2_maxProxies);
		
		var edgeCount:uint = 2 * m_proxyCount;
		
		var lowerValues:Array = new Array();
		var upperValues:Array = new Array();
		ComputeBounds(lowerValues, upperValues, aabb);
		
		for (var axis:int = 0; axis < 2; ++axis)
		{
			var bounds:Array = m_bounds[axis];
			var lowerIndex:uint;
			var upperIndex:uint;
			var lowerIndexOut:Array = [lowerIndex];
			var upperIndexOut:Array = [upperIndex];
			Query(lowerIndexOut, upperIndexOut, lowerValues[axis], upperValues[axis], bounds, edgeCount, axis);
			lowerIndex = lowerIndexOut[0];
			upperIndex = upperIndexOut[0];
			
			// Replace memmove calls
			//memmove(bounds + upperIndex + 2, bounds + upperIndex, (edgeCount - upperIndex) * sizeof(b2Bound));
			var tArr:Array = new Array();
			var j:int;
			var tEnd:int = edgeCount - upperIndex
			var tBound1:b2Bound;
			var tBound2:b2Bound;
			// make temp array
			for (j = 0; j < tEnd; j++){
				tArr[j] = new b2Bound();
				tBound1 = tArr[j];
				tBound2 = bounds[upperIndex+j];
				tBound1.value = tBound2.value;
				tBound1.proxyId = tBound2.proxyId;
				tBound1.stabbingCount = tBound2.stabbingCount;
			}
			// move temp array back in to bounds
			tEnd = tArr.length;
			var tIndex:int = upperIndex+2;
			for (j = 0; j < tEnd; j++){
				//bounds[tIndex+j] = tArr[j];
				tBound2 = tArr[j];
				tBound1 = bounds[tIndex+j]
				tBound1.value = tBound2.value;
				tBound1.proxyId = tBound2.proxyId;
				tBound1.stabbingCount = tBound2.stabbingCount;
			}
			//memmove(bounds + lowerIndex + 1, bounds + lowerIndex, (upperIndex - lowerIndex) * sizeof(b2Bound));
			// make temp array
			tArr = new Array();
			tEnd = upperIndex - lowerIndex;
			for (j = 0; j < tEnd; j++){
				tArr[j] = new b2Bound();
				tBound1 = tArr[j];
				tBound2 = bounds[lowerIndex+j];
				tBound1.value = tBound2.value;
				tBound1.proxyId = tBound2.proxyId;
				tBound1.stabbingCount = tBound2.stabbingCount;
			}
			// move temp array back in to bounds
			tEnd = tArr.length;
			tIndex = lowerIndex+1;
			for (j = 0; j < tEnd; j++){
				//bounds[tIndex+j] = tArr[j];
				tBound2 = tArr[j];
				tBound1 = bounds[tIndex+j]
				tBound1.value = tBound2.value;
				tBound1.proxyId = tBound2.proxyId;
				tBound1.stabbingCount = tBound2.stabbingCount;
			}
			
			// The upper index has increased because of the lower bound insertion.
			++upperIndex;
			
			// Copy in the new bounds.
			bounds[lowerIndex].value = lowerValues[axis];
			bounds[lowerIndex].proxyId = proxyId;
			bounds[upperIndex].value = upperValues[axis];
			bounds[upperIndex].proxyId = proxyId;
			
			bounds[lowerIndex].stabbingCount = lowerIndex == 0 ? 0 : bounds[lowerIndex-1].stabbingCount;
			bounds[upperIndex].stabbingCount = bounds[upperIndex-1].stabbingCount;
			
			// Adjust the stabbing count between the new bounds.
			for (index = lowerIndex; index < upperIndex; ++index)
			{
				bounds[index].stabbingCount++;
			}
			
			// Adjust the all the affected bound indices.
			for (index = lowerIndex; index < edgeCount + 2; ++index)
			{
				var proxy2:b2Proxy = m_proxyPool[ bounds[index].proxyId ];
				if (bounds[index].IsLower())
				{
					proxy2.lowerBounds[axis] = index;
				}
				else
				{
					proxy2.upperBounds[axis] = index;
				}
			}
		}
		
		++m_proxyCount;
		
		//b2Settings.b2Assert(m_queryResultCount < b2Settings.b2_maxProxies);
		
		for (var i:int = 0; i < m_queryResultCount; ++i)
		{
			if (ShouldCollide(proxyId, m_queryResults[i]) == false){
				continue;
			}
			var pair:b2Pair = m_pairManager.Add(proxyId, m_queryResults[i]);
			if (pair == null)
			{
				continue;
			}
			
			// The Add command may return an old pair, which should not happen here.
			//b2Settings.b2Assert(pair.IsReceived() == false);
			pair.userData = m_pairCallback.PairAdded(proxy.userData, m_proxyPool[m_queryResults[i]].userData);
			pair.SetReceived();
		}
		
		// Prepare for next query.
		m_queryResultCount = 0;
		IncrementTimeStamp();
		
		return proxyId;
	}
	
	public function DestroyProxy(proxyId:uint){
		if (proxyId == b2Pair.b2_nullProxy)
		{
			//b2Settings.b2Assert(false);
			return;
		}
		
		// Flush the pair buffer.
		Flush();
		
		var proxy:b2Proxy = m_proxyPool[ proxyId ];
		var edgeCount:uint = 2 * m_proxyCount;
		
		for (var axis:int = 0; axis < 2; ++axis)
		{
			var bounds:Array = m_bounds[axis];
			
			var lowerIndex:uint = proxy.lowerBounds[axis];
			var upperIndex:uint = proxy.upperBounds[axis];
			var lowerValue:uint = bounds[lowerIndex].value;
			var upperValue:uint = bounds[upperIndex].value;
			
			// replace memmove calls
			//memmove(bounds + lowerIndex, bounds + lowerIndex + 1, (upperIndex - lowerIndex - 1) * sizeof(b2Bound));
			var tArr:Array = new Array();
			var j:int;
			var tEnd:int = upperIndex - lowerIndex - 1;
			var tBound1:b2Bound;
			var tBound2:b2Bound;
			// make temp array
			for (j = 0; j < tEnd; j++){
				tArr[j] = new b2Bound();
				tBound1 = tArr[j];
				tBound2 = bounds[lowerIndex+1+j];
				tBound1.value = tBound2.value;
				tBound1.proxyId = tBound2.proxyId;
				tBound1.stabbingCount = tBound2.stabbingCount;
			}
			// move temp array back in to bounds
			tEnd = tArr.length;
			var tIndex:int = lowerIndex;
			for (j = 0; j < tEnd; j++){
				//bounds[tIndex+j] = tArr[j];
				tBound2 = tArr[j];
				tBound1 = bounds[tIndex+j]
				tBound1.value = tBound2.value;
				tBound1.proxyId = tBound2.proxyId;
				tBound1.stabbingCount = tBound2.stabbingCount;
			}
			//memmove(bounds + upperIndex-1, bounds + upperIndex + 1, (edgeCount - upperIndex - 1) * sizeof(b2Bound));
			// make temp array
			tArr = new Array();
			tEnd = edgeCount - upperIndex - 1;
			for (j = 0; j < tEnd; j++){
				tArr[j] = new b2Bound();
				tBound1 = tArr[j];
				tBound2 = bounds[upperIndex+1+j];
				tBound1.value = tBound2.value;
				tBound1.proxyId = tBound2.proxyId;
				tBound1.stabbingCount = tBound2.stabbingCount;
			}
			// move temp array back in to bounds
			tEnd = tArr.length;
			tIndex = upperIndex-1;
			for (j = 0; j < tEnd; j++){
				//bounds[tIndex+j] = tArr[j];
				tBound2 = tArr[j];
				tBound1 = bounds[tIndex+j]
				tBound1.value = tBound2.value;
				tBound1.proxyId = tBound2.proxyId;
				tBound1.stabbingCount = tBound2.stabbingCount;
			}
			
			// Fix bound indices.
			tEnd = edgeCount - 2;
			for (var index:uint = lowerIndex; index < tEnd; ++index)
			{
				var proxy2:b2Proxy = m_proxyPool[ bounds[index].proxyId ];
				if (bounds[index].IsLower())
				{
					proxy2.lowerBounds[axis] = index;
				}
				else
				{
					proxy2.upperBounds[axis] = index;
				}
			}
			
			// Fix stabbing count.
			tEnd = upperIndex - 1;
			for (var index2:int = lowerIndex; index2 < tEnd; ++index2)
			{
				bounds[index2].stabbingCount--;
			}
			
			// Query for pairs to be removed. lowerIndex and upperIndex are not needed.
			// make lowerIndex and upper output using an array and do this for others if compiler doesn't pick them up
			Query(new Array(), new Array(), lowerValue, upperValue, bounds, edgeCount - 2, axis);
		}
		
		//b2Settings.b2Assert(m_queryResultCount < b2Settings.b2_maxProxies);
		
		for (var i:int = 0; i < m_queryResultCount; ++i)
		{
			//b2Settings.b2Assert(proxy.IsValid() && m_proxyPool[m_queryResults[i]].IsValid());
			
			var other:b2Proxy = m_proxyPool[ m_queryResults[i] ];
			var pairUserData:* = m_pairManager.Remove(proxyId, m_queryResults[i]);
			m_pairCallback.PairRemoved(proxy.userData, other.userData, pairUserData);
		}
		
		// Prepare for next query.
		m_queryResultCount = 0;
		IncrementTimeStamp();
		
		// Invalidate the proxy.
		proxy.userData = null;
		proxy.overlapCount = b2_invalid;
		
		// Return the proxy to the pool.
		proxy.SetNext(m_freeProxy);
		m_freeProxy = proxyId;
		--m_proxyCount;
	}


	// Call MoveProxy as many times as you like, then when you are done
	// call Flush to finalized the proxy pairs (for your time step).
	public function MoveProxy(proxyId:uint, aabb:b2AABB){
		var index:uint;
		var bound:b2Bound;
		var prevEdge:b2Bound
		var nextEdge:b2Bound
		var nextProxyId:uint;
		var nextProxy:b2Proxy;
		
		if (proxyId == b2Pair.b2_nullProxy || b2Settings.b2_maxProxies <= proxyId)
		{
			return;
		}
		
		if (aabb.IsValid() == false)
		{
			//b2Settings.b2Assert(false);
			return;
		}
		
		var edgeCount:uint = 2 * m_proxyCount;
		
		var proxy:b2Proxy = m_proxyPool[ proxyId ];
		var lowerValues:Array = new Array();
		var upperValues:Array = new Array();
		ComputeBounds(lowerValues, upperValues, aabb);
		
		for (var axis:uint = 0; axis < 2; ++axis)
		{
			var bounds:Array = m_bounds[axis];
			
			var lowerIndex:uint = proxy.lowerBounds[axis];
			var upperIndex:uint = proxy.upperBounds[axis];
			
			var lowerValue:uint = lowerValues[axis];
			var upperValue:uint = upperValues[axis];
			
			var deltaLower:int = lowerValue - bounds[lowerIndex].value;
			var deltaUpper:int = upperValue - bounds[upperIndex].value;
			
			bounds[lowerIndex].value = lowerValue;
			bounds[upperIndex].value = upperValue;
			
			//
			// Expanding adds overlaps
			//
			
			// Should we move the lower bound down?
			if (deltaLower < 0)
			{
				index = lowerIndex;
				while (index > 0 && lowerValue < bounds[index-1].value)
				{
					bound = bounds[index];
					prevEdge = bounds[index - 1];
					
					var prevProxyId:uint = prevEdge.proxyId;
					var prevProxy:b2Proxy = m_proxyPool[ prevEdge.proxyId ];
					
					prevEdge.stabbingCount++;
					
					if (prevEdge.IsUpper() == true)
					{
						if (TestOverlap(proxy, prevProxy))
						{
							AddBufferedPair(proxyId, prevProxyId);
						}
						
						prevProxy.upperBounds[axis]++;
						bound.stabbingCount++;
					}
					else
					{
						prevProxy.lowerBounds[axis]++;
						bound.stabbingCount--;
					}
					
					proxy.lowerBounds[axis]--;
					
					// swap
					//var temp:b2Bound = bound;
					//bound = prevEdge;
					//prevEdge = temp;
					bound.Swap(prevEdge);
					//b2Math.b2Swap(bound, prevEdge);
					--index;
				}
			}
			
			// Should we move the upper bound up?
			if (deltaUpper > 0)
			{
				index = upperIndex;
				while (index < edgeCount-1 && bounds[index+1].value <= upperValue)
				{
					bound = bounds[ index ];
					nextEdge = bounds[ index + 1 ];
					nextProxyId = nextEdge.proxyId;
					nextProxy = m_proxyPool[ nextProxyId ];
					
					nextEdge.stabbingCount++;
					
					if (nextEdge.IsLower() == true)
					{
						if (TestOverlap(proxy, nextProxy))
						{
							AddBufferedPair(proxyId, nextProxyId);
						}
						
						nextProxy.lowerBounds[axis]--;
						bound.stabbingCount++;
					}
					else
					{
						nextProxy.upperBounds[axis]--;
						bound.stabbingCount--;
					}
					
					proxy.upperBounds[axis]++;
					// swap
					//var temp:b2Bound = bound;
					//bound = nextEdge;
					//nextEdge = temp;
					bound.Swap(nextEdge);
					//b2Math.b2Swap(bound, nextEdge);
					index++;
				}
			}
			
			//
			// Shrinking removes overlaps
			//
			
			// Should we move the lower bound up?
			if (deltaLower > 0)
			{
				index = lowerIndex;
				while (index < edgeCount-1 && bounds[index+1].value <= lowerValue)
				{
					bound = bounds[ index ];
					nextEdge = bounds[ index + 1 ];
					
					nextProxyId = nextEdge.proxyId;
					nextProxy = m_proxyPool[ nextProxyId ];
					
					nextEdge.stabbingCount--;
					
					if (nextEdge.IsUpper())
					{
						RemoveBufferedPair(proxyId, nextProxyId);
						
						nextProxy.upperBounds[axis]--;
						bound.stabbingCount--;
					}
					else
					{
						nextProxy.lowerBounds[axis]--;
						bound.stabbingCount++;
					}
					
					proxy.lowerBounds[axis]++;
					// swap
					//var temp:b2Bound = bound;
					//bound = nextEdge;
					//nextEdge = temp;
					bound.Swap(nextEdge);
					//b2Math.b2Swap(bound, nextEdge);
					index++;
				}
			}
			
			// Should we move the upper bound down?
			if (deltaUpper < 0)
			{
				index = upperIndex;
				while (index > 0 && upperValue < bounds[index-1].value)
				{
					bound = bounds[index];
					prevEdge = bounds[index - 1];
					
					prevProxyId = prevEdge.proxyId;
					prevProxy = m_proxyPool[ prevProxyId ];
					
					prevEdge.stabbingCount--;
					
					if (prevEdge.IsLower() == true)
					{
						RemoveBufferedPair(proxyId, prevProxyId);
						
						prevProxy.lowerBounds[axis]++;
						bound.stabbingCount--;
					}
					else
					{
						prevProxy.upperBounds[axis]++;
						bound.stabbingCount++;
					}
					
					proxy.upperBounds[axis]--;
					// swap
					//var temp:b2Bound = bound;
					//bound = prevEdge;
					//prevEdge = temp;
					bound.Swap(prevEdge);
					//b2Math.b2Swap(bound, prevEdge);
					index--;
				}
			}
		}
	}
	
	public function Flush(){
		var i:int;
		
		var removeCount:int = 0;
		
		for (i = 0; i < m_pairBufferCount; ++i)
		{
			
			var pair:b2Pair = m_pairManager.Find(m_pairBuffer[i].proxyId1, m_pairBuffer[i].proxyId2);
			//b2Settings.b2Assert(pair.IsBuffered());
			
			var proxy1:b2Proxy = m_proxyPool[ pair.proxyId1 ];
			var proxy2:b2Proxy = m_proxyPool[ pair.proxyId2 ];
			
			//b2Settings.b2Assert(proxy1.IsValid());
			//b2Settings.b2Assert(proxy2.IsValid());
			
			if (pair.IsRemoved())
			{
				//b2Settings.b2Assert(TestOverlap(proxy1, proxy2) == false);
				
				if (pair.IsReceived())
				{
					m_pairCallback.PairRemoved(proxy1.userData, proxy2.userData, pair.userData);
				}
				
				// Store the ids so we can actually remove the pair below.
				m_pairBuffer[removeCount].proxyId1 = pair.proxyId1;
				m_pairBuffer[removeCount].proxyId2 = pair.proxyId2;
				removeCount++;
			}
			else
			{
				//b2Settings.b2Assert(TestOverlap(proxy1, proxy2) == true);
				pair.ClearBuffered();
				
				if (pair.IsReceived() == false)
				{
					pair.userData = m_pairCallback.PairAdded(proxy1.userData, proxy2.userData);
					pair.SetReceived();
				}
			}
		}
		
		for (i = 0; i < removeCount; ++i)
		{
			m_pairManager.Remove(m_pairBuffer[i].proxyId1, m_pairBuffer[i].proxyId2);
		}
		
		m_pairBufferCount = 0;
	}

	// Query an AABB for overlapping proxies, returns the user data and
	// the count, up to the supplied maximum count.
	public function QueryAABB(aabb:b2AABB, userData:*, maxCount:int):int{
		var lowerValues:Array = new Array();
		var upperValues:Array = new Array();
		ComputeBounds(lowerValues, upperValues, aabb);
		
		var lowerIndex:uint;
		var upperIndex:uint;
		var lowerIndexOut:Array = [lowerIndex];
		var upperIndexOut:Array = [upperIndex];
		Query(lowerIndexOut, upperIndexOut, lowerValues[0], upperValues[0], m_bounds[0], 2*m_proxyCount, 0);
		Query(lowerIndexOut, upperIndexOut, lowerValues[1], upperValues[1], m_bounds[1], 2*m_proxyCount, 1);
		
		//b2Settings.b2Assert(m_queryResultCount < b2Settings.b2_maxProxies);
		
		var count:int = 0;
		for (var i:int = 0; i < m_queryResultCount && count < maxCount; ++i, ++count)
		{
			//b2Settings.b2Assert(m_queryResults[i] < b2Settings.b2_maxProxies);
			var proxy:b2Proxy = m_proxyPool[ m_queryResults[i] ];
			//b2Settings.b2Assert(proxy.IsValid());
			userData[i] = proxy.userData;
		}
		
		// Prepare for next query.
		m_queryResultCount = 0;
		IncrementTimeStamp();
		
		return count;
	}

	public function Validate(){
		var pair:b2Pair;
		var proxy1:b2Proxy;
		var proxy2:b2Proxy;
		var overlap:Boolean;
		
		for (var axis:int = 0; axis < 2; ++axis)
		{
			var bounds:b2Bound = m_bounds[axis];
			
			var pointCount:uint = 2 * m_proxyCount;
			var stabbingCount:uint = 0;
			
			for (var i:uint = 0; i < pointCount; ++i)
			{
				var bound:b2Bound = bounds[i];
				if (i > 0)
				{
					var prevEdge:b2Bound = bounds[i - 1];
					//b2Settings.b2Assert(prevEdge.value <= bound.value);
				}
				
				var proxyId:uint = bound.proxyId;
				
				//b2Settings.b2Assert(proxyId != b2Pair.b2_nullProxy);
				
				var proxy:b2Proxy = m_proxyPool[ bound.proxyId ];
				
				//b2Settings.b2Assert(proxy.IsValid());
				
				if (bound.IsLower() == true)
				{
					//b2Settings.b2Assert(proxy.lowerBounds[axis] == i);
					stabbingCount++;
				}
				else
				{
					//b2Settings.b2Assert(proxy.upperBounds[axis] == i);
					stabbingCount--;
				}
				
				//b2Settings.b2Assert(bound.stabbingCount == stabbingCount);
			}
		}
		
		var pairs:Array = m_pairManager.GetPairs();
		var pairCount:uint = m_pairManager.GetCount();
		//b2Settings.b2Assert(m_pairBufferCount <= pairCount);
		
		// replace std::sort
		//std::sort(m_pairBuffer, m_pairBuffer + m_pairBufferCount);
		m_pairBuffer.sortOn(["proxyId1", "proxyId2"], Array.NUMERIC);
		
		for (var j:int = 0; j < m_pairBufferCount; ++j)
		{
			if (j > 0)
			{
				//b2Settings.b2Assert(Equals(m_pairBuffer[i], m_pairBuffer[i-1]) == false);
			}
			
			pair = m_pairManager.Find(m_pairBuffer[j].proxyId1, m_pairBuffer[j].proxyId2);
			//b2Settings.b2Assert(pair.IsBuffered());
			
			proxy1 = m_proxyPool[ pair.proxyId1 ];
			proxy2 = m_proxyPool[ pair.proxyId2 ];
			
			//b2Settings.b2Assert(proxy1.IsValid() == true);
			//b2Settings.b2Assert(proxy2.IsValid() == true);
			
			overlap = TestOverlap(proxy1, proxy2);
			
			if (pair.IsRemoved() == true)
			{
				//b2Settings.b2Assert(overlap == false);
			}
			else
			{
				//b2Settings.b2Assert(overlap == true);
			}
		}
		
		for (var k:int = 0; k < pairCount; ++k)
		{
			pair = pairs[ k ];
			
			proxy1 = m_proxyPool[ pair.proxyId1 ];
			proxy2 = m_proxyPool[ pair.proxyId2 ];
			
			//b2Settings.b2Assert(proxy1.IsValid() == true);
			//b2Settings.b2Assert(proxy2.IsValid() == true);
			
			overlap = TestOverlap(proxy1, proxy2);
			
			if (pair.IsBuffered())
			{
				if (pair.IsRemoved() == true)
				{
					//b2Settings.b2Assert(overlap == false);
				}
				else
				{
					//b2Settings.b2Assert(overlap == true);
				}
			}
			else
			{
				//b2Settings.b2Assert(overlap == true);
			}
		}
	}
	public function ValidatePairs(){
		
		var pairCount:uint = m_pairManager.GetCount();
		//b2Settings.b2Assert(m_pairBufferCount <= pairCount);
		
		// replace std::sort
		//std::sort(m_pairBuffer, m_pairBuffer + m_pairBufferCount);
		m_pairBuffer.sortOn(["proxyId1", "proxyId2"], Array.NUMERIC);
		
		for (var i:int = 0; i < m_pairBufferCount; ++i)
		{
			if (i > 0)
			{
				//b2Settings.b2Assert(Equals(m_pairBuffer[i], m_pairBuffer[i-1]) == false);
			}
			
			var pair:b2Pair = m_pairManager.Find(m_pairBuffer[i].proxyId1, m_pairBuffer[i].proxyId2);
			//b2Settings.b2Assert(pair.IsBuffered());
			
			var proxy1:b2Proxy = m_proxyPool[ pair.proxyId1 ];
			var proxy2:b2Proxy = m_proxyPool[ pair.proxyId2 ];
			
			//b2Settings.b2Assert(proxy1.IsValid() == true);
			//b2Settings.b2Assert(proxy2.IsValid() == true);
		}
	}

//private:
	private function ComputeBounds(lowerValues:Array, upperValues:Array, aabb:b2AABB){
		//var minVertex:b2Vec2 = b2Math.b2ClampV(aabb.minVertex, m_worldAABB.minVertex, m_worldAABB.maxVertex);
		var minVertexX:Number = aabb.minVertex.x;
		var minVertexY:Number = aabb.minVertex.y;
		minVertexX = b2Math.b2Min(minVertexX, m_worldAABB.maxVertex.x);
		minVertexY = b2Math.b2Min(minVertexY, m_worldAABB.maxVertex.y);
		minVertexX = b2Math.b2Max(minVertexX, m_worldAABB.minVertex.x);
		minVertexY = b2Math.b2Max(minVertexY, m_worldAABB.minVertex.y);
		
		//var maxVertex:b2Vec2 = b2Math.b2ClampV(aabb.maxVertex, m_worldAABB.minVertex, m_worldAABB.maxVertex);
		var maxVertexX:Number = aabb.maxVertex.x;
		var maxVertexY:Number = aabb.maxVertex.y;
		maxVertexX = b2Math.b2Min(maxVertexX, m_worldAABB.maxVertex.x);
		maxVertexY = b2Math.b2Min(maxVertexY, m_worldAABB.maxVertex.y);
		maxVertexX = b2Math.b2Max(maxVertexX, m_worldAABB.minVertex.x);
		maxVertexY = b2Math.b2Max(maxVertexY, m_worldAABB.minVertex.y);
		
		// Bump lower bounds downs and upper bounds up. This ensures correct sorting of
		// lower/upper bounds that would have equal values.
		// TODO_ERIN implement fast float to uint16 conversion.
		lowerValues[0] = uint(m_quantizationFactor.x * (minVertexX - m_worldAABB.minVertex.x)) & (b2Settings.USHRT_MAX - 1);
		upperValues[0] = (uint(m_quantizationFactor.x * (maxVertexX - m_worldAABB.minVertex.x))& 0x0000ffff) | 1;
		
		lowerValues[1] = uint(m_quantizationFactor.y * (minVertexY - m_worldAABB.minVertex.y)) & (b2Settings.USHRT_MAX - 1);
		upperValues[1] = (uint(m_quantizationFactor.y * (maxVertexY - m_worldAABB.minVertex.y))& 0x0000ffff) | 1;
	}

	// As proxies are created and moved, many pairs are created and destroyed. Even worse, the same
	// pair may be added and removed multiple times in a single time step of the physics engine. To reduce
	// traffic in the pair manager, we try to avoid destroying pairs in the pair manager until all pairs 
	// end of the physics step is called. This is done by buffering all the RemovePair requests. AddPair
	// requests are processed immediately because we need the hash table entry for quick lookup.
	// 
	// All user user callbacks are delay until the buffered pairs are confirmed in Flush.
	// This is very important because the user callbacks may be very expensive and client logic
	// may be harmed if pairs are added and removed within the same time step.

	// Buffer a pair for addition.
	// We may add a pair that is not in the pair manager or pair buffer.
	// We may add a pair that is already in the pair manager and pair buffer.
	// If the added pair is not a new pair, then it must be in the pair buffer (because RemovePair was called).
	private function AddBufferedPair(id1:uint, id2:uint){
		//b2Settings.b2Assert(m_proxyPool[id1].IsValid() && m_proxyPool[id2].IsValid());
		if (ShouldCollide(id1, id2) == false)
		{
			return;
		}
		var pair:b2Pair = m_pairManager.Add(id1, id2);
		
		if (pair == null)
		{
			return;
		}
		
		// If this pair is not in the pair buffer ...
		if (pair.IsBuffered() == false)
		{
			// This must be a new pair.
			//b2Settings.b2Assert(pair.IsReceived() == false);
			
			// If there is room in the pair buffer ...
			if (m_pairBufferCount < b2Settings.b2_maxPairs)
			{
				// Add it to the pair buffer.
				pair.SetBuffered();
				m_pairBuffer[m_pairBufferCount].proxyId1 = pair.proxyId1;
				m_pairBuffer[m_pairBufferCount].proxyId2 = pair.proxyId2;
				++m_pairBufferCount;
			}
			
			//b2Settings.b2Assert(m_pairBufferCount <= m_pairManager.GetCount());
		}
		
		// Confirm this pair for the subsequent call to Flush.
		pair.ClearRemoved();
	}
	private function RemoveBufferedPair(id1:uint, id2:uint){
		//b2Settings.b2Assert(m_proxyPool[id1].IsValid() && m_proxyPool[id2].IsValid());
		
		var pair:b2Pair = m_pairManager.Find(id1, id2);
		
		if (pair == null)
		{
			return;
		}
		
		// If this pair is not in the pair buffer ...
		if (pair.IsBuffered() == false)
		{
			// This must be an old pair.
			//b2Settings.b2Assert(pair.IsReceived());
			
			if (m_pairBufferCount < b2Settings.b2_maxPairs)
			{
				pair.SetBuffered();
				m_pairBuffer[m_pairBufferCount].proxyId1 = pair.proxyId1;
				m_pairBuffer[m_pairBufferCount].proxyId2 = pair.proxyId2;
				++m_pairBufferCount;
			}
			
			//b2Settings.b2Assert(m_pairBufferCount <= m_pairManager.GetCount());
		}
		
		pair.SetRemoved();
	}

	private function TestOverlap(p1:b2Proxy, p2:b2Proxy):Boolean{
		
		for (var axis:int = 0; axis < 2; ++axis)
		{
			var bounds:Array = m_bounds[axis];
			
			if (bounds[p1.lowerBounds[axis]].value > bounds[p2.upperBounds[axis]].value)
				return false;
			
			if (bounds[p1.upperBounds[axis]].value < bounds[p2.lowerBounds[axis]].value)
				return false;
		}
		
		return true;
	}

	private function Query(lowerQueryOut:Array, upperQueryOut:Array, lowerValue:uint, upperValue:uint, bounds:Array, edgeCount:uint, axis:int){
		
		var lowerQuery:uint = BinarySearch(bounds, edgeCount, lowerValue);
		var upperQuery:uint = BinarySearch(bounds, edgeCount, upperValue);
		
		// Easy case: lowerQuery <= lowerIndex(i) < upperQuery
		// Solution: search query range for min bounds.
		for (var j:uint = lowerQuery; j < upperQuery; ++j)
		{
			if (bounds[j].IsLower())
			{
				IncrementOverlapCount(bounds[j].proxyId);
			}
		}
		
		// Hard case: lowerIndex(i) < lowerQuery < upperIndex(i)
		// Solution: use the stabbing count to search down the bound array.
		if (lowerQuery > 0)
		{
			var i:int = lowerQuery - 1;
			var s:int = bounds[i].stabbingCount;
			
			// Find the s overlaps.
			while (s)
			{
				//b2Settings.b2Assert(i >= 0);
				
				if (bounds[i].IsLower())
				{
					var proxy:b2Proxy = m_proxyPool[ bounds[i].proxyId ];
					if (lowerQuery <= proxy.upperBounds[axis])
					{
						IncrementOverlapCount(bounds[i].proxyId);
						--s;
					}
				}
				--i;
			}
		}
		
		lowerQueryOut[0] = lowerQuery;
		upperQueryOut[0] = upperQuery;
	}


	private function IncrementOverlapCount(proxyId:uint){
		var proxy:b2Proxy = m_proxyPool[ proxyId ];
		if (proxy.timeStamp < m_timeStamp)
		{
			proxy.timeStamp = m_timeStamp;
			proxy.overlapCount = 1;
		}
		else
		{
			proxy.overlapCount = 2;
			//b2Settings.b2Assert(m_queryResultCount < b2Settings.b2_maxProxies);
			m_queryResults[m_queryResultCount] = proxyId;
			++m_queryResultCount;
		}
	}
	private function IncrementTimeStamp(){
		if (m_timeStamp == b2Settings.USHRT_MAX)
		{
			for (var i:uint = 0; i < b2Settings.b2_maxProxies; ++i)
			{
				m_proxyPool[i].timeStamp = 0;
			}
			m_timeStamp = 1;
		}
		else
		{
			++m_timeStamp;
		}
	}

//public:
	public var m_pairManager:b2PairManager = new b2PairManager();

	public var m_proxyPool:Array = new Array(b2Settings.b2_maxPairs);
	public var m_freeProxy:uint;

	public var m_pairBuffer:Array = new Array(b2Settings.b2_maxPairs);
	public var m_pairBufferCount:int;

	public var m_bounds:Array = new Array(2*b2Settings.b2_maxProxies);

	public var m_pairCallback:b2PairCallback;
	public var m_queryResults:Array = new Array(b2Settings.b2_maxProxies);
	public var m_queryResultCount:int;

	public var m_worldAABB:b2AABB;
	public var m_quantizationFactor:b2Vec2 = new b2Vec2();
	public var m_proxyCount:int;
	public var m_timeStamp:uint;

	static public var s_validate:Boolean = false;
	
	static public const b2_invalid:uint = b2Settings.USHRT_MAX;
	static public const b2_nullEdge:uint = b2Settings.USHRT_MAX;


	public function ShouldCollide(id1:int, id2:int):Boolean
	{
		//b2Settings.b2Assert(id1 < b2Settings.b2_maxProxies);
		//b2Settings.b2Assert(id2 < b2Settings.b2_maxProxies);
		var p1:b2Proxy = m_proxyPool[ id1 ];
		var p2:b2Proxy = m_proxyPool[ id2 ];
		
		if (p1.groupIndex == p2.groupIndex && p1.groupIndex != 0)
		{
			return p1.groupIndex > 0;
		}
		
		var doCollide:Boolean = (p1.maskBits & p2.categoryBits) != 0 && (p1.categoryBits & p2.maskBits) != 0;
		return doCollide;
	}
	
	static public function LessThanBP(pair1:b2BufferedPair, pair2:b2BufferedPair):Boolean
	{
		if (pair1.proxyId1 < pair2.proxyId1)
			return true;
		
		if (pair1.proxyId1 == pair2.proxyId1)
		{
			return pair1.proxyId2 < pair2.proxyId2;
		}
		
		return false;
	}
	
	static public function Equals(pair1:b2BufferedPair, pair2:b2BufferedPair):Boolean
	{
		if (pair1.proxyId1 == pair2.proxyId1 && pair1.proxyId2 == pair2.proxyId2)
		{
			return true;
		}

		return false;
	}

	static public function BinarySearch(bounds:Array, count:int, value:uint):uint
	{
		var low:int = 0;
		var high:int = count - 1;
		while (low <= high)
		{
			var mid:int = ((low + high) / 2);
			if (bounds[mid].value > value)
			{
				high = mid - 1;
			}
			else if (bounds[mid].value < value)
			{
				low = mid + 1;
			}
			else
			{
				return uint(mid);
			}
		}
		
		return uint(low);
	}
	
	
};
}