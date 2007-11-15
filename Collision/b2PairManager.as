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

// The pair manager is used by the broad-phase to quickly add/remove/find pairs
// of overlapping proxies. It is based closely on code provided by Pierre Terdiman.
// http://www.codercorner.com/IncrementalSAP.txt

package Engine.Collision{


import Engine.Common.*
import Engine.Common.Math.*


public class b2PairManager
{
//public:
	public function b2PairManager(){
		var i:uint;
		//b2Settings.b2Assert(b2Math.b2IsPowerOfTwo(b2Pair.b2_tableCapacity) == true);
		//b2Settings.b2Assert(b2Pair.b2_tableCapacity >= b2Settings.b2_maxPairs);
		m_hashTable = new Array(b2Pair.b2_tableCapacity);
		for (i = 0; i < b2Pair.b2_tableCapacity; ++i)
		{
			m_hashTable[i] = b2Pair.b2_nullPair;
		}
		m_next = new Array(b2Settings.b2_maxPairs);
		for (i = 0; i < b2Settings.b2_maxPairs; ++i)
		{
			m_next[i] = b2Pair.b2_nullPair;
		}
		m_pairs = new Array(b2Settings.b2_maxPairs);
		for (i = 0; i < b2Settings.b2_maxPairs; ++i)
		{
			m_pairs[i] = new b2Pair();
		}
		m_pairCount = 0;
	}
	//~b2PairManager();

	// Add a pair and return the new pair. If the pair already exists,
	// no new pair is created and the old one is returned.
	public function Add(proxyId1:uint, proxyId2:uint):b2Pair{
		
		if (proxyId1 > proxyId2){
			var temp:uint = proxyId1;
			proxyId1 = proxyId2;
			proxyId2 = temp;
			//b2Math.b2Swap(p1, p2);
		}
		
		var hash:uint = Hash(proxyId1, proxyId2) & b2Pair.b2_tableMask;
		
		var pairIndex:int = FindHash(proxyId1, proxyId2, hash);
		var pair:b2Pair = pairIndex != b2Settings.USHRT_MAX? m_pairs[ pairIndex ]:null;
		
		if (pair != null)
		{
			return pair;
		}
		
		if (m_pairCount == b2Settings.b2_maxPairs)
		{
			//b2Settings.b2Assert(false);
			return null;
		}
		
		pair = m_pairs[ m_pairCount ];
		pair.proxyId1 = proxyId1;
		pair.proxyId2 = proxyId2;
		pair.status = 0;
		pair.userData = null;
		
		m_next[m_pairCount] = m_hashTable[hash];
		m_hashTable[hash] = m_pairCount;
		
		++m_pairCount;
		
		return pair;
	}

	// Remove a pair, return the pair's userData.
	public function Remove(proxyId1:uint, proxyId2:uint):*{
		if (proxyId1 > proxyId2){
			var temp:uint = proxyId1;
			proxyId1 = proxyId2;
			proxyId2 = temp;
			//b2Math.b2Swap(proxyId1, proxyId2);
		}
		
		var hash:uint = Hash(proxyId1, proxyId2) & b2Pair.b2_tableMask;
		
		var pairIndex:int = FindHash(proxyId1, proxyId2, hash);
		var pair:b2Pair = pairIndex != b2Settings.USHRT_MAX? m_pairs[ pairIndex ]:null;
		if (pair == null)
		{
			return null;
		}
		
		var userData:* = pair.userData;
		
		//b2Settings.b2Assert(pair.proxyId1 == proxyId1);
		//b2Settings.b2Assert(pair.proxyId2 == proxyId2);
		
		//var pairIndex:int = int(pair - m_pairs);
		//b2Settings.b2Assert(pairIndex < m_pairCount);
		
		// Remove the pair from the hash table.
		var index:uint = m_hashTable[hash];
		//b2Settings.b2Assert(index != b2Pair.b2_nullPair);
		
		var previous:uint = b2Pair.b2_nullPair;
		while (index != pairIndex)
		{
			previous = index;
			index = m_next[index];
		}
		
		if (previous != b2Pair.b2_nullPair)
		{
			//b2Settings.b2Assert(m_next[previous] == pairIndex);
			m_next[previous] = m_next[pairIndex];
		}
		else
		{
			m_hashTable[hash] = m_next[pairIndex];
		}
		
		// We now move the last pair into spot of the
		// pair being removed. We need to fix the hash
		// table indices to support the move.
		
		var lastPairIndex:uint = m_pairCount - 1;
		
		// If the removed pair is the last pair, we are done.
		if (lastPairIndex == pairIndex)
		{
			--m_pairCount;
			return userData;
		}
		
		// Remove the last pair from the hash table.
		var last:b2Pair = m_pairs[ lastPairIndex ];
		var lastHash:uint = Hash(last.proxyId1, last.proxyId2) & b2Pair.b2_tableMask;
		
		index = m_hashTable[lastHash];
		//b2Settings.b2Assert(index != b2Pair.b2_nullPair);
		
		previous = b2Pair.b2_nullPair;
		while (index != lastPairIndex)
		{
			previous = index;
			index = m_next[index];
		}
		
		if (previous != b2Pair.b2_nullPair)
		{
			//b2Settings.b2Assert(m_next[previous] == lastPairIndex);
			m_next[previous] = m_next[lastPairIndex];
		}
		else
		{
			m_hashTable[lastHash] = m_next[lastPairIndex];
		}
		
		// Copy the last pair into the remove pair's spot.
		// _COPY_ ...
		m_pairs[pairIndex].userData = m_pairs[lastPairIndex].userData;
		m_pairs[pairIndex].proxyId1 = m_pairs[lastPairIndex].proxyId1;
		m_pairs[pairIndex].proxyId2 = m_pairs[lastPairIndex].proxyId2;
		m_pairs[pairIndex].status = m_pairs[lastPairIndex].status;
		
		// Insert the last pair into the hash table
		m_next[pairIndex] = m_hashTable[lastHash];
		m_hashTable[lastHash] = pairIndex;
		
		--m_pairCount;
		
		return userData;
	}

	public function Find(proxyId1:uint, proxyId2:uint):b2Pair{
		
		if (proxyId1 > proxyId2){
			var temp:uint = proxyId1;
			proxyId1 = proxyId2;
			proxyId2 = temp;
			//b2Math.b2Swap(proxyId1, proxyId2);
		}
		
		var hash:uint = Hash(proxyId1, proxyId2) & b2Pair.b2_tableMask;
		
		var index:uint = m_hashTable[hash];
		while (index != b2Pair.b2_nullPair && Equals(m_pairs[index], proxyId1, proxyId2) == false)
		{
			index = m_next[index];
		}
		if (index == b2Pair.b2_nullPair)
		{
			return null;
		}
		
		//b2Settings.b2Assert(index < m_pairCount);
		
		return m_pairs[ index ];
	}

	public function GetCount():int { return m_pairCount; }
	public function GetPairs():Array { return m_pairs; }

//private:
	private function FindHash(proxyId1:uint, proxyId2:uint, hash:uint):int{
		var index:uint = m_hashTable[hash];
		
		while( index != b2Pair.b2_nullPair && Equals(m_pairs[index], proxyId1, proxyId2) == false)
		{
			index = m_next[index];
		}
		
		if ( index == b2Pair.b2_nullPair )
		{
			return b2Settings.USHRT_MAX;
		}
		
		//b2Settings.b2Assert(index < m_pairCount);
		
		return index;
	}

//public:
	public var m_pairs:Array;
	public var m_pairCount:int;

	public var m_hashTable:Array;
	public var m_next:Array;
	
	
// static
	// Thomas Wang's hash, see: http://www.concentric.net/~Ttwang/tech/inthash.htm
	static public function Hash(proxyId1:uint, proxyId2:uint):uint
	{
		var key:uint = ((proxyId2 << 16) & 0xffff0000) | proxyId1;
		key = ~key + ((key << 15) & 0xFFFF8000);
		key = key ^ ((key >> 12) & 0x000fffff);
		key = key + ((key << 2) & 0xFFFFFFFC);
		key = key ^ ((key >> 4) & 0x0fffffff);
		key = key * 2057;
		key = key ^ ((key >> 16) & 0x0000ffff);
		return key;
	}
	
	static public function Equals(pair:b2Pair, proxyId1:uint, proxyId2:uint):Boolean
	{
		return (pair.proxyId1 == proxyId1 && pair.proxyId2 == proxyId2);
	}
	
};

}