package Box2D.Collision 
{
	import Box2D.Common.Math.b2Vec2;
	public interface IBroadPhase 
	{
		/// Create a proxy with an initial AABB. Pairs are not reported until
		/// UpdatePairs is called.
		function CreateProxy(aabb:b2AABB, userData:*):*;
		
		/// Destroy a proxy. It is up to the client to remove any pairs.
		function DestroyProxy(proxy:*):void;
		
		/// Call MoveProxy as many times as you like, then when you are done
		/// call UpdatePairs to finalized the proxy pairs (for your time step).
		function MoveProxy(proxy:*, aabb:b2AABB):void;
		
		function TestOverlap(proxyA:*, proxyB:*):Boolean;
		
		/// Get user data from a proxy. Returns null if the proxy is invalid.
		function GetUserData(proxy:*):*;
		
		/// Get the fat AABB for a proxy.
		function GetFatAABB(proxy:*):b2AABB;
		
		/// Get the number of proxies.
		function GetProxyCount():int;
		
		/// Update the pairs. This results in pair callbacks. This can only add pairs.
		function UpdatePairs(callback:Function):void;
		
		/// Query an AABB for overlapping proxies. The callback class
		/// is called for each proxy that overlaps the supplied AABB.
		function Query(callback:Function, aabb:b2AABB):void;
		
		/// For debugging, throws in invariants have been broken
		function Validate():void;
		
		/// Give the broadphase a chance for structural optimizations
		function Rebalance(iterations:int):void;
	}
	
}