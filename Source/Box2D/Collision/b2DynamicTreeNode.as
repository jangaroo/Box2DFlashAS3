package Box2D.Collision 
{
	
	/**
	 * A node in the dynamic tree. The client does not interact with this directly.
	 */
	public class b2DynamicTreeNode 
	{
		public function IsLeaf():Boolean
		{
			return child1 == null;
		}
		
		public var userData:*;
		public var aabb:b2AABB;
		public var parent:b2DynamicTreeNode;
		public var child1:b2DynamicTreeNode;
		public var child2:b2DynamicTreeNode;
	}
	
}