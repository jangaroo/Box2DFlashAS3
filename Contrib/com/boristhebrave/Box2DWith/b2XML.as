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
	import flash.utils.Dictionary;
	
	/**
	 * Contains functions for loading Box2D shapes, bodies and joints from a simple XML format.
	 * 
	 * <p>The XML format is formally defined in using <a href="http://relaxng.org/">Relax NG</a> in the file
	 * <a href="box2d.rng">box2d.rng</a> found in the same directory as this class.
	 * An <a href="http://www.w3.org/XML/Schema">XML Schema</a> file is also <a href="box2d.xsd">available</a>, autotranslated by 
	 * <a href="http://www.thaiopensource.com/relaxng/trang.html">Trang</a>.</p>
	 * 
	 * <p>Simply stated, the XML format has a root &lt;world/&gt; element. Inside that, there are various body and joint elements, 
	 * and inside each body element is various shape elements, thus matching Box2Ds design layout quite closely.
	 * See the methods loadShapeDef, loadBodyDef, and loadJointDef for the details on how each element is formed.</p>
	 * 
	 * <p>In general, attribute names match exactly the corresponding Box2D property, and has the same defaults as Box2D.
	 * Reasonable values are generated for certain properties when not specified, in the same manner as the various
	 * Initialize functions of Box2D. Joint anchors can be specified in either world or local co-ordinates, either single
	 * or jointly, though this implementation will not prevent you from overspecifying attributes.</p>
	 * 
	 * <p>It is expected that in most cases, you will not want to use the XML to fully define worlds using loadWorld, as
	 * this library doesn't provide any mechanism for handling other data, such as the appearance of a body. Instead, you
	 * can use the various loading functions to synthesize your own XML format containing parts of the Box2D XML specification.
	 * Or you can simply use this class as a more consise and portable way of writing out defintions, and deal with defining
	 * the world in your own way.</p>
	 * 
	 * @see #loadShapeDef()
	 * @see #loadBodyDef()
	 * @see #loadJointDef()
	 */
	public class b2XML 
	{
		
		public static const ns:Namespace = new Namespace("com.boristhebrave.Box2DWith:b2XML");
		/**
		 * Loads a Number from a given attribute.
		 * @param	attribute	An attribute list of zero or one attributes to parse into a Number.
		 * @param	defacto		The default number to use in case there is no attribute or it is not a valid Number.
		 * @return The parsed Number, or defacto.
		 */
		public static function loadFloat(attribute:XMLList, defacto:Number):Number
		{
			try
			{
				if (attribute.length())
					return parseFloat(attribute);
			}
			catch (error:Error)
			{
				
			}
			return defacto;
		}
		
		/**
		 * Loads a angle from a given attribute. Angles are read like loadFloat, with an optional letter suffix.
		 * "d" indicates the angle is in degrees, not radians.
		 * @param	attribute	An attribute list of zero or one attributes to parse into a Number.
		 * @param	defacto		The default number to use in case there is no attribute or it is not a valid Number.
		 * @return The parsed Number, or defacto.
		 */
		public static function loadAngle(attribute:XMLList, defacto:Number):Number
		{
			try
			{
				if (attribute.length())
				{
					var s:String = attribute.toString();
					var f:Number = parseFloat(attribute);
					if (s.charAt(s.length - 1) == "d")
						f *= Math.PI / 180;
					return f;
				}
			}
			catch (error:Error)
			{
				
			}
			return defacto;
		}
		
		/**
		 * Loads a int from a given attribute.
		 * @param	attribute	An attribute list of zero or one attributes to parse into a int.
		 * @param	defacto		The default number to use in case there is no attribute or it is not a valid int.
		 * @return The parsed int, or defacto.
		 */
		public static function loadInt(attribute:XMLList, defacto:int):int
		{
			try
			{
				if (attribute.length())
					return parseInt(attribute);
			}
			catch (error:Error)
			{
				
			}
			return defacto;
		}
		
		/**
		 * Loads a Boolean from a given attribute. Only the value "true" is recognized as true. Everything else is false.
		 * @param	attribute	An attribute list of zero or one attributes to parse into a Boolean.
		 * @param	defacto		The default number to use in case there is no attribute.
		 * @return The parsed Boolean, or defacto.
		 */
		public static function loadBool(attribute:XMLList, defacto:Boolean = false):Boolean
		{
			try
			{
				if (attribute.length())
					return attribute.toString()=="true";
			}
			catch (error:Error)
			{
				
			}
			return defacto;
		}
		
		/**
		 * Loads a String from a given attribute.
		 * @param	attribute	An attribute list of zero or one attributes.
		 * @param	defacto		The default number to use in case there is no attribute.
		 * @return The attribute, if it exists, or defacto.
		 */
		public static function loadString(attribute:XMLList, defacto:String = ""):String
		{
			try
			{
				if (attribute.length())
					return attribute.toString();
			}
			catch (error:Error)
			{
				
			}
			return defacto;
		}
		
		/**
		 * Loads a b2Vec2 from a given attribute. Vectors are stored as space delimited Numbers, e.g. "1.5 2.3".
		 * @param	attribute	An attribute list of zero or one attributes to parse into a b2Vec2.
		 * @param	defacto		The default number to use in case there is no attribute or it is not a valid b2Vec2.
		 * @return The parsed b2Vec2, or defacto.
		 */
		public static function loadVec2(attribute:XMLList, defacto:b2Vec2 = null):b2Vec2
		{
			try
			{
				var a:Array = String(attribute).split(/\s+/);
				return b2Vec2.Make.apply(null, a);
			}
			catch (error:Error)
			{
				
			}
			return defacto;
		}
		
		/**
		 * Reads a b2FixtureDef from xml. The element type and it's children are ignored, only
		 * attributes are read.
		 * 
		 * <p>The following attributes are supported:</p>
		 * <pre>
		 * density        float
		 * friction       float
		 * isSensor       Boolean
		 * userData       String
		 * categoryBits   int
		 * maskBits       int
		 * groupIndex     int</pre>
		 * @param	base A fixture definition to use as the default when an XML attribute is missing.
		 */
		public static function loadFixtureDef(xml:XML, base:b2FixtureDef = null):b2FixtureDef
		{
			var to:b2FixtureDef = new b2FixtureDef();
			if (base) {
				to.density = base.density;
				to.filter = base.filter.Copy();
				to.friction = base.friction;
				to.isSensor = base.isSensor;
				to.restitution = base.restitution;
				to.userData = base.userData;
			}
			if (xml.@density.length()) {
				var density:String = xml.@density.toString().toLowerCase();
				switch(density) {
					case "":
					case "default":
						break;
					default:
						to.density = parseFloat(xml.@density);
				}
			}
			to.restitution = loadFloat(xml.@restitution, to.restitution);
			to.friction = loadFloat(xml.@friction, to.friction);
			to.isSensor = loadBool(xml.@isSensor, to.isSensor);
			if (xml.@userData.length())
				to.userData = xml.@userData;
			to.filter.categoryBits = loadInt(xml.@categoryBits, to.filter.categoryBits);
			to.filter.maskBits = loadInt(xml.@maskBits, to.filter.maskBits);
			to.filter.groupIndex = loadInt(xml.@groupIndex, to.filter.groupIndex);
			return to;
		}
		
		/**
		 * Converts an XML element into a b2Shape.
		 * 
		 * <p>The following elements/usages are recognized:</p>
		 * <pre>
		 * &lt;circle radius="0." x="0." y="0."/&gt;
		 * 	    b2CircleDef 
		 * &lt;circle radius="0." localPosition="0. 0."/&gt;
		 * 	    b2CircleDef    
		 * &lt;polygon&gt;
		 * 	&lt;vertex x="0." y="0."/&gt;
		 *  &lt;vertex x="0." y="0."/&gt;
		 *  &lt;vertex x="0." y="0."/&gt;
		 * &lt;/polygon&gt;
		 * 	    b2PolygonDef
		 * &lt;box x="0." y="0." width="0." height="0." angle="0."/&gt;
		 * 	    b2PolygonDef formed into an OBB.
		 * &lt;box left="" right="" top="" bottom=""/&gt;
		 * 	    b2PolygonDef formed into an AABB.
		 * 	    height and width can substitute for one of top/bottom and one of left/right.</pre>
		 * 
		 * 
		 * @param	shape An XML element in the above format
		 * @return	The corresponding b2Shape
		 */
		public static function loadShape(shape:XML):b2Shape
		{
			switch(shape.localName()) {
				case "circle":
					var circle:b2CircleShape = new b2CircleShape();
					circle.SetRadius(loadFloat(shape.@radius, 0));
					var localPosition:b2Vec2 =  new b2Vec2(loadFloat(shape.@x, 0), loadFloat(shape.@y, 0));
					circle.SetLocalPosition(loadVec2(shape.@localPosition, localPosition));
					return circle;
				case "polygon":
					var vertices:XMLList = shape.ns::vertex;
					if (vertices.length() == 0)
						vertices = shape.vertex;
					var vertices2:Vector.<b2Vec2> = new Vector.<b2Vec2>();
					for (var i:int = 0; i < vertices.length(); i++) {
						vertices2[i] = new b2Vec2(parseFloat(vertices[i].@x),
						                          parseFloat(vertices[i].@y));
					}
					return b2PolygonShape.AsVector(vertices2);
				case "box":
					//Standard format
					var width:Number = parseFloat(shape.@width);
					var height:Number = parseFloat(shape.@height);
					var x:Number = loadFloat(shape.@x, 0);
					var y:Number = loadFloat(shape.@y,0);
					var angle:Number = loadFloat(shape.@angle,0);
					//Alt format
					if (!shape.@angle.length()){
						if (shape.@left.length() && shape.@right.length())
						{
							x = (parseFloat(shape.@right) + parseFloat(shape.@left)) / 2;
							width = parseFloat(shape.@right) - parseFloat(shape.@left);
						}
						if (shape.@left.length() && shape.@width.length())
						{
							x = parseFloat(shape.@left) + width / 2;
						}
						if (shape.@right.length() && shape.@width.length())
						{
							x = parseFloat(shape.@right) - width / 2;
						}
						if (shape.@top.length() && shape.@bottom.length())
						{
							y = (parseFloat(shape.@bottom) + parseFloat(shape.@top)) / 2;
							height = parseFloat(shape.@bottom) - parseFloat(shape.@top);
						}
						if (shape.@top.length() && shape.@height.length())
						{
							y = parseFloat(shape.@top) + height / 2;
						}
						if (shape.@bottom.length() && shape.@height.length())
						{
							y = parseFloat(shape.@bottom) - height / 2;
						}
					}
					return b2PolygonShape.AsOrientedBox(width/2, height/2, new b2Vec2(x, y), angle);
			}
			return null;
		}
		
		/**
		 * Converts a &lt;body/&gt; element into a b2BodyDef.
		 * 
		 * <p>The following attributes are recognized, corresponding directly
		 * to the b2BodyDef properties:</p>
		 * <pre>
		 * allowSleep       Boolean
		 * angle            Number
		 * angularDamping   Number
		 * fixedRotation    Boolean
		 * bullet           Boolean
		 * awake            Boolean
		 * linearDamping    Number
		 * x                Number
		 * y                Number
		 * position         b2Vec2
		 * userData         &#x2A; 	</pre>
		 * @param	body An XML element in the above format.
		 * @param	base A body definition to use as the default when an XML attribute is missing.
		 * @return The specified b2BodyDef.
		 */
		public static function loadBodyDef(body:XML, base:b2BodyDef = null):b2BodyDef
		{
			var bodyDef:b2BodyDef = new b2BodyDef();
			if (base)
			{
				bodyDef.allowSleep = base.allowSleep;
				bodyDef.angle = base.angle;
				bodyDef.angularDamping = base.angularDamping;
				bodyDef.fixedRotation = base.fixedRotation;
				bodyDef.bullet = base.bullet;
				bodyDef.awake = base.awake;
				bodyDef.linearDamping = base.linearDamping;
				bodyDef.position = base.position.Copy();
				bodyDef.userData = base.userData;
			}
			bodyDef.allowSleep = loadBool(body.@allowSleep, bodyDef.allowSleep);
			bodyDef.angle = loadAngle(body.@angle, bodyDef.angle);
			bodyDef.angularDamping = loadFloat(body.@angularDamping, bodyDef.angularDamping);
			bodyDef.fixedRotation = loadBool(body.@fixedRotation, bodyDef.fixedRotation);
			bodyDef.bullet = loadBool(body.@isBullet, bodyDef.bullet);
			bodyDef.awake = loadBool(body.@awake, bodyDef.awake);
			bodyDef.linearDamping = loadFloat(body.@linearDamping, bodyDef.linearDamping);
			bodyDef.position.x = loadFloat(body.@x, bodyDef.position.x);
			bodyDef.position.y = loadFloat(body.@y, bodyDef.position.y);
			bodyDef.position = loadVec2(body.@position, bodyDef.position);
			if (body.@userData.length())
				bodyDef.userData = String(body.@userData);
			return bodyDef;
		}
		
		/**
		 * Creates a body from a &lt;body/&gt; element with nested shape elements, using the definitions from loadBodyDef and loadShapeDef.
		 * @param	xml	The &lt;body/&gt; element to parse.
		 * @param	world	The world to create the body from.
		 * @param	bodyDef	The base body definition to use for defaults.
		 * @param	fixtureDef A fixture definition to use for defaults.
		 * @return A newly created body in world.
		 * @see #loadBodyDef()
		 * @see #loadShapeDef()
		 */
		public static function loadBody(xml:XML, world:b2World, bodyDef:b2BodyDef = null, fixtureDef:b2FixtureDef = null):b2Body
		{
			var bd:b2BodyDef = loadBodyDef(xml, bodyDef);
			if (!bd)
				return null;
			var body:b2Body = world.CreateBody(bd);
			for each(var el:XML in xml.*)
			{
				var fd:b2FixtureDef = loadFixtureDef(el, fixtureDef);
				var shape:b2Shape;
				if (el.localName() == "fixture")
				{
					for each(var shapeXML:XML in el.*)
					{
						shape = loadShape(shapeXML);
						if (shape)
						{
							fd.shape = shape;
							body.CreateFixture(fd);
						}
					}
				}else {
					shape = loadShape(el);
					if (shape)
					{
						fd.shape = shape;
						body.CreateFixture(fd);
					}
				}
			}
			return body;
		}
		
		/**
		 * Reads common joint def attributes from xml.
		 */
		private static function assignJointDefFromXML(xml:XML, to:b2JointDef, bodyA:b2Body, bodyB:b2Body, base:b2JointDef = null):void {
			if (base)
			{
				to.userData = base.userData;
				to.bodyA = base.bodyA;
				to.bodyB = base.bodyB;
				to.collideConnected = base.collideConnected;
				if (base is b2GearJointDef && to is b2GearJointDef)
				{
					(to as b2GearJointDef).joint1 = (base as b2GearJointDef).joint1;
					(to as b2GearJointDef).joint2 = (base as b2GearJointDef).joint2;
				}
			}
			to.bodyA = bodyA;
			to.bodyB = bodyB;
			var localAnchorA:b2Vec2 = loadVec2(xml.attribute("local-anchorA"));
			var localAnchorB:b2Vec2 = loadVec2(xml.attribute("local-anchorB"));
			var worldAnchorA:b2Vec2 = loadVec2(xml.attribute("world-anchorA"));
			var worldAnchorB:b2Vec2 = loadVec2(xml.attribute("world-anchorB"));
			var worldAnchor:b2Vec2 = loadVec2(xml.attribute("world-anchor"));
			if (worldAnchor)
				worldAnchorA = worldAnchorB = worldAnchor;
			if (worldAnchorA)
				localAnchorA = to.bodyA.GetLocalPoint(worldAnchorA);
			if (worldAnchorA)
				localAnchorA = to.bodyB.GetLocalPoint(worldAnchorB);
			if (!localAnchorA)
				localAnchorA = new b2Vec2();
			if (!localAnchorB)
				localAnchorB = new b2Vec2();
			{
				if (to is b2DistanceJointDef)
				{
					(to as b2DistanceJointDef).localAnchorA = localAnchorA;
					(to as b2DistanceJointDef).localAnchorB = localAnchorB;
				}
				if (to is b2PrismaticJointDef)
				{
					(to as b2PrismaticJointDef).localAnchorA = localAnchorA;
					(to as b2PrismaticJointDef).localAnchorB = localAnchorB;
				}
				if (to is b2RevoluteJointDef)
				{
					(to as b2RevoluteJointDef).localAnchorA = localAnchorA;
					(to as b2RevoluteJointDef).localAnchorB = localAnchorB;
				}
				if (to is b2PulleyJointDef)
				{
					(to as b2PulleyJointDef).localAnchorA = localAnchorA;
					(to as b2PulleyJointDef).localAnchorB = localAnchorB;
				}
			}
			if (xml.@collideConnected.length())
				to.collideConnected = xml.@collideConnected == "true";
		}
		
		/**
		 * Converts an XML element into a b2JointDef.
		 * 
		 * <p>The following elements and attributes are recognized:</p>
		 * <pre>
		 * &lt;gear/&gt;    b2GearJointDef
		 *         ratio           Number
		 *         joint1          String    (resolved)
		 *         joint2          String    (resolved)
		 * 
		 * &lt;prismatic/&gt; b2PrismaticJointDef
		 *         motorSpeed      Number
		 *         maxMotorForce   Number
		 *         enableMotor     Boolean   (automatically set)
		 *         lower           Number
		 *         upper           Number
		 *         enableLimits    Boolean   (automatically set)
		 *         referenceAngle  Number    (automatically set)
		 *         world-axis      b2Vec2
		 *         local-axisA     b2Vec2
		 * 
		 * &lt;revolute/&gt;    b2RevoluteJointDef
		 *         motorSpeed      Number
		 *         maxMotorTorque  Number
		 *         enableMotor     Boolean   (automatically set)
		 *         lower           Number
		 *         upper           Number
		 *         enableLimits    Boolean   (automatically set)
		 *         referenceAngle  Number    (automatically set)
		 * 
		 * &lt;distance/&gt;    b2DistanceJointDef
		 *         dampingRatio    Number
		 *         frequencyHz     Number
		 *         length          Number    (automatically set)
		 * 
		 * &lt;pulley/&gt; b2PulleyJointDef
		 *         ratio           Number
		 *         maxLength-a     Number
		 *         maxLength-b     Number
		 *         world-ground    b2Vec2
		 *         world-groundA   b2Vec2
		 *         world-groundB   b2Vec2
		 *         length-a        Number    (automatically set)
		 *         length-b        Number    (automatically set)
		 * 
		 * &lt;mouse/&gt;    b2MouseJointDef
		 *         dampingRatio    Number
		 *         frequencyHz     Number
		 *         maxForce        Number
		 *         target          b2Vec2 </pre>
		 * 
		 * <p>Additionally, all elements support the following attributes:</p>
		 * <pre>
		 * body1             String          (resolved)
		 * body2             String          (resolved)
		 * world-anchor      b2Vec2
		 * world-anchorA     b2Vec2
		 * world-anchorB     b2Vec2
		 * local-anchorA     b2Vec2
		 * local-anchorB     b2Vec2
		 * collideConnected  Boolean </pre>
		 * 
		 * <p>Note that if the joint does not have a well defined body from bodyA/bodyB or via providing base,
		 * then world co-ordinates cannot be used, except for the ground anchors of a pulley joint.</p>
		 * 
		 * @param	joint An XML element in the above format
		 * @param	resolver A function mapping strings to b2Bodys (and b2Joint).
		 * This is used so that the bodyA and bodyB (and joint1 and joint2 from &lt;gear/&gt;) can get resolved
		 * to the correct references. You can avoid using this if these properties are not defined, and providing them via base.
		 * @param	base A joint definition to use as the default when an XML attribute is missing.
		 * @return	The corresponding b2ShapeDef
		 */
		public static function loadJointDef(joint:XML, resolver:Function, base:b2JointDef=null):b2JointDef
		{
			//Determine the bodies involved.
			var bodyA:b2Body;
			var bodyB:b2Body;
			if (base && base.bodyA)
				bodyA = base.bodyA;
			if (base && base.bodyB)
				bodyB = base.bodyB;
			if (joint.attribute("bodyA").length())
				bodyA = resolver(String(joint.attribute("bodyA")));
			if (joint.attribute("bodyB").length())
				bodyB = resolver(String(joint.attribute("bodyB")));
			switch(joint.localName()) {
				case "gear":
					var gearDef:b2GearJointDef = new b2GearJointDef();
					assignJointDefFromXML(joint, gearDef, bodyA, bodyB, base);
					gearDef.collideConnected = true;
					gearDef.ratio = loadFloat(joint.@ratio, 1);
					gearDef.joint1 = resolver(String(joint.@joint1));
					gearDef.joint2 = resolver(String(joint.@joint2));
					return gearDef;
				case "prismatic":
					var prismaticDef:b2PrismaticJointDef = new b2PrismaticJointDef();
					assignJointDefFromXML(joint, prismaticDef, bodyA, bodyB, base);
					// Parse from joint
					// Motor stuff
					prismaticDef.motorSpeed = loadFloat(joint.@motorSpeed, prismaticDef.motorSpeed);
					prismaticDef.maxMotorForce = loadFloat(joint.@maxMotorForce, Number.POSITIVE_INFINITY);
					prismaticDef.enableMotor = loadBool(joint.@enableMotor, joint.@motorSpeed.length() || joint.@maxMotorForce.length());
					// Limit stuff
					prismaticDef.lowerTranslation = loadFloat(joint.@lower, Number.NEGATIVE_INFINITY);
					prismaticDef.upperTranslation = loadFloat(joint.@upper, Number.POSITIVE_INFINITY);
					prismaticDef.enableLimit = loadBool(joint.@enableLimit, joint.@lower.length() || joint.@upper.length());
					//Joint stuff
					prismaticDef.referenceAngle = loadFloat(joint.@referenceAngle, bodyB.GetAngle() - bodyA.GetAngle());
						
					var worldAxis:b2Vec2 = loadVec2(joint.attribute("world-axis"));
					var localAxis:b2Vec2 = loadVec2(joint.attribute("local-axis-a"));
					if (worldAxis)
						localAxis = bodyA.GetLocalVector(worldAxis);
					localAxis.Normalize();
					prismaticDef.localAxisA = localAxis;
					
					return prismaticDef;
				case "revolute":
					var revoluteDef:b2RevoluteJointDef = new b2RevoluteJointDef();
					assignJointDefFromXML(joint, revoluteDef, bodyA, bodyB, base);
					// Motor stuff
					revoluteDef.motorSpeed = loadFloat(joint.@motorSpeed, revoluteDef.motorSpeed);
					revoluteDef.maxMotorTorque = loadFloat(joint.@maxMotorTorque, Number.POSITIVE_INFINITY);
					revoluteDef.enableMotor = loadBool(joint.@enableMotor, joint.@motorSpeed.length() || joint.@maxMotorTorque.length());
					// Limit stuff
					revoluteDef.lowerAngle = loadFloat(joint.@lower, Number.NEGATIVE_INFINITY);
					revoluteDef.upperAngle = loadFloat(joint.@upper, Number.POSITIVE_INFINITY);
					revoluteDef.enableLimit = loadBool(joint.@enableLimit, joint.@lower.length() || joint.@upper.length());
					revoluteDef.referenceAngle = loadFloat(joint.@referenceAngle, bodyB.GetAngle() - bodyA.GetAngle());
					return revoluteDef;
				case "distance":
					var distanceDef:b2DistanceJointDef = new b2DistanceJointDef();
					assignJointDefFromXML(joint, distanceDef, bodyA, bodyB, base);
					distanceDef.dampingRatio = loadFloat(joint.@dampingRatio, distanceDef.dampingRatio);
					distanceDef.frequencyHz = loadFloat(joint.@frequencyHz, distanceDef.frequencyHz);
					if (joint.@length.length())
					{
						distanceDef.length = loadFloat(joint.@length.length(), 0);
					}
					else
					{
						distanceDef.length = b2Math.SubtractVV(bodyA.GetWorldPoint(distanceDef.localAnchorA), bodyB.GetWorldPoint(distanceDef.localAnchorB)).Length();
					}
					return distanceDef;
				case "pulley":
					var pulleyDef:b2PulleyJointDef = new b2PulleyJointDef()
					assignJointDefFromXML(joint, pulleyDef, bodyA, bodyB, base);
					
					pulleyDef.ratio = loadFloat(joint.@ratio, 1);
					pulleyDef.maxLengthA = loadFloat(joint.attribute("maxLengthA"), pulleyDef.maxLengthA);
					pulleyDef.maxLengthB = loadFloat(joint.attribute("maxLengthB"), pulleyDef.maxLengthB);
					pulleyDef.groundAnchorA = loadVec2(joint.attribute("world-groundA"), pulleyDef.groundAnchorA);
					pulleyDef.groundAnchorB = loadVec2(joint.attribute("world-groundB"), pulleyDef.groundAnchorB);
					var ground:b2Vec2 = loadVec2(joint.attribute("world-ground"));
					if (ground)
					{
						pulleyDef.groundAnchorA = ground.Copy();
						pulleyDef.groundAnchorB = ground.Copy();
					}
					if (joint.@lengthA.length())
					{
						pulleyDef.lengthA = loadFloat(joint.@lengthA, pulleyDef.lengthA);
					}
					else
					{
						pulleyDef.lengthA = b2Math.SubtractVV(bodyA.GetWorldPoint(pulleyDef.localAnchorA), pulleyDef.groundAnchorA).Length();
					}
					if (joint.@lengthB.length())
					{
						pulleyDef.lengthB = loadFloat(joint.@lengthB, pulleyDef.lengthB);
					}
					else
					{
						pulleyDef.lengthB = b2Math.SubtractVV(bodyB.GetWorldPoint(pulleyDef.localAnchorB), pulleyDef.groundAnchorB).Length();
					}
					return pulleyDef;
				case "mouse":
					var mouseDef:b2MouseJointDef = new b2MouseJointDef();
					assignJointDefFromXML(joint, mouseDef, bodyA, bodyB, base);
					mouseDef.dampingRatio = loadFloat(joint.@dampingRatio, mouseDef.dampingRatio);
					mouseDef.frequencyHz = loadFloat(joint.@frequencyHz, mouseDef.frequencyHz);
					mouseDef.maxForce = loadFloat(joint.@maxForce, mouseDef.maxForce);
					mouseDef.target = loadVec2(joint.@target, mouseDef.target);
					return mouseDef;
			}
			return null;
		}
	
		/**
		 * Loads a world given a XML defintion. 
		 * 
		 * <p>xml is expected to a &lt;world&gt; element with child &lt;body&gt; and joint elements as specified in
		 * loadBodyDef and loadShapeDef. &lt;body/&gt; elements should have children shape elements as 
		 * specified in loadShapeDef.</p>
		 * 
		 * <p>Both body and joint elements can have an id attribute that gives a string identifier
		 * to be later resolved for use with the body1 and body2 attributes of joints,
		 * and joint1 and joint2 attribute of gear joints.</p>
		 * 
		 * @param	xml		A <world/> element in the above format.
		 * @param	world	A world to load into. Unlike other load functions, this function does not create an object from scratch.
		 * @param	bodyDef	A body definition to use for defaults.
		 * @param	fixtureDef A fixture definition to use for defaults.
		 * @param	jointDef A joint definition to use for defaults.
		 * @return A function you can use to resolve the loaded elements, as defined in loadJointDef.
		 * @see #loadJointDef
		 */
		public static function loadWorld(xml:XML, world:b2World, bodyDef:b2BodyDef = null, fixtureDef:b2FixtureDef = null, jointDef:b2JointDef=null):Function
		{
			var idMapping:Object = { };
			var resolver:Function = function(id:String):*
			{
				return idMapping[id];
			}
			for each(var element:XML in xml.*)
			{
				if (element.localName() == "body")
				{
					var body:b2Body = loadBody(element, world, bodyDef, fixtureDef);
					if (element.@id.length())
						idMapping[String(element.@id)] = body;
				}
				else
				{
					var jd:b2JointDef = loadJointDef(element, resolver, jointDef);
					var joint:b2Joint = world.CreateJoint(jd);
					if (element.@id.length())
						idMapping[String(element.@id)] = joint;
				}
			}
			return resolver;
		}
		
		/** Inverse of loadFloat */
		public static function saveFloat(xml:XMLList, value:Number):void
		{
			xml[0] = value.toString();
		}
		
		/** Inverse of loadFloat, omitting the attribute for the default value. */
		public static function saveFloat2(xml:XMLList, value:Number, defacto:Number = 0.0):void
		{
			if (Math.abs(value-defacto) < 4 * Number.MIN_VALUE)
				delete xml[0];
			else 
				xml[0] = value.toString();
		}
		
		/** Inverse of loadBool. */
		public static function saveBool(xml:XMLList, value:Boolean):void
		{
			xml[0] = value ? "true" : "false"
		}
		
		/** Inverse of loadBool, omitting the attribute for the default value. */
		public static function saveBool2(xml:XMLList, value:Boolean, defacto:Boolean = false):void
		{
			if (value == defacto)
				delete xml[0]
			else
				xml[0] = value ? "true" : "false"
		}
		
		/** 
		 * Inverse of loadVec2
		 * @param defacto If provided, and value equals defacto, then don't write the attribute.
		 */
		public static function saveVec2(xml:XMLList, value:b2Vec2, defacto:b2Vec2 = null):void
		{
			if (defacto && (Math.abs(value.x - defacto.x) < 4 * Number.MIN_VALUE) && (Math.abs(value.y - defacto.y) < 4 * Number.MIN_VALUE))
				delete xml[0]
			else
				xml[0] = value.x.toString() + " " + value.y.toString();
		}
		
		/** 
		 * Inverse of loadFixtureDef
		 * @param base If provided, and then for each field of def, don't write the attribute if it matches base.
		 */
		public static function saveFixtureDef(xml:XML, def:b2FixtureDef, base:b2FixtureDef = null):void
		{
		}
		
		/** Inverse of loadShape */
		public static function saveShape(shape:b2Shape):XML
		{
			switch(shape.GetType())
			{
				case b2Shape.e_circleShape:
				{
					var circle:b2CircleShape = shape as b2CircleShape;
					var circleXML:XML = <circle/>
					circleXML.@radius = circle.GetRadius();
					saveVec2(circleXML.@localPosition, circle.GetLocalPosition());
					return circleXML;
				}
				case b2Shape.e_polygonShape:
				{
					var poly:b2PolygonShape = shape as b2PolygonShape;
					var boxArray:Array = b2Geometry.DetectBox(shape);
					if (boxArray)
					{
						var boxXML:XML = <box/>
						saveFloat2(boxXML.@x, boxArray[2].x);
						saveFloat2(boxXML.@y, boxArray[2].y);
						saveFloat(boxXML.@width, boxArray[0]*2);
						saveFloat(boxXML.@height, boxArray[1]*2);
						saveFloat2(boxXML.@angle, boxArray[3]);
						return boxXML;
					}else {
						var polyXML:XML = <polygon/>
						var vertices:Vector.<b2Vec2> = poly.GetVertices();
						for (var i:int = 0; i < poly.GetVertexCount(); i++)
						{
							polyXML.appendChild(<vertex x={vertices[i].x} y={vertices[i].y}/>);
						}
						return polyXML;
					}
				}
				default:
				return null;
			}
		}
		
		/** Inverse of loadBodyDef */
		public static function saveBodyDef(body:XML, bodyDef:b2BodyDef, base:b2BodyDef = null):void
		{
			if (!base)
			{
				saveBool(body.@allowSleep, bodyDef.allowSleep);
				saveFloat(body.@angle, bodyDef.angle);
				saveFloat(body.@angularDamping, bodyDef.angularDamping);
				saveBool(body.@fixedRotation, bodyDef.fixedRotation);
				saveBool(body.@bullet, bodyDef.bullet);
				saveBool(body.@awake, bodyDef.awake);
				saveFloat(body.@linearDamping, bodyDef.linearDamping);
				saveFloat(body.@x, bodyDef.position.x);
				saveFloat(body.@y, bodyDef.position.y);
				if (bodyDef.userData)
				{
					body.@userData = String(bodyDef.userData);
				}else {
					delete body.@userData;
				}
			}else {
				saveBool2(body.@allowSleep, bodyDef.allowSleep, base.allowSleep);
				saveFloat2(body.@angle, bodyDef.angle, base.angle);
				saveFloat2(body.@angularDamping, bodyDef.angularDamping, base.angularDamping);
				saveBool2(body.@fixedRotation, bodyDef.fixedRotation, base.fixedRotation);
				saveBool2(body.@bullet, bodyDef.bullet, base.bullet);
				saveBool2(body.@awake, bodyDef.awake, base.awake);
				saveFloat2(body.@linearDamping, bodyDef.linearDamping, base.linearDamping);
				saveFloat2(body.@x, bodyDef.position.x, base.position.x);
				saveFloat2(body.@y, bodyDef.position.y, base.position.y);
				if (bodyDef.userData && bodyDef.userData != base.userData)
				{
					body.@userData = String(bodyDef.userData);
				}else {
					delete body.@userData;
				}
			}
		}

		/**
		 * Saves a world to XML.
		 * This doesn't currently support joints.
		 */
		public static function saveWorld(world:b2World, bodyDef:b2BodyDef = null, fixtureDef:b2FixtureDef = null, jointDef:b2JointDef = null):XML
		{
			var xml:XML = <world/>;
			var map:Dictionary/*b2Body,XML*/ = new Dictionary();
			var i:int = 0;
			for (var body:b2Body = world.GetBodyList(); body; body = body.GetNext())
			{
				var bXml:XML = <body id={"b"+i.toString()}/>;
				map[body] = bXml;
				saveBodyDef(bXml, body.GetDefinition(), bodyDef);
				xml.appendChild(bXml);
				i++;
			}
			// TODO:
			for (var joint:b2Joint = world.GetJointList(); joint; joint = joint.GetNext())
			{
			}
			return xml;
		}
	}
	
}