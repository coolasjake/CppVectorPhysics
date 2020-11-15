namespace Physics
{
#include <vector>

	using namespace std;

	///<summary> Class which stores x and y values, and performs mathematical functions relating to distance and direction. </summary>
	class pVector
	{
	public:
		float x = 0;
		float y = 0;

		///<summary> Create a vector with the default values (0, 0). </summary>
		pVector() {

		}

		///<summary> Create a vector with the specified x and y values. </summary>
		pVector(float X, float Y) {
			x = X;
			y = Y;
		}

		///<summary> Add two vectors together (x1 + x2, y1 + y2). </summary>
		pVector operator + (pVector const &other) {
			return pVector(x + other.x, y + other.y);
		}

		///<summary> Subtract a vector from another (x1 - x2, y1 - y2). </summary>
		pVector operator - (pVector const &other) {
			return pVector(x - other.x, y - other.y);
		}

		///<summary> Multiply a vector by a value (x * val, y * val).  </summary>
		pVector operator * (float value) {
			return pVector(x * value, y * value);
		}

		///<summary> Multiply two vectors together (x1 * x2, y1 * y2) - warning: this is rarely used in physics calculations. </summary>
		pVector operator * (pVector const &other) {
			return pVector(x * other.x, y * other.y);
		}

		///<summary> The size of the vector, i.e. distance from (0, 0). </summary>
		float Magnitude() {
			return sqrtf(powf(x, 2) + powf(y, 2));
		}

		///<summary> Returns the magnitude of the vector before square-root function.
		///Can be used when comparing which vector is larger, since the square-root function is unnecessary and very expensive. </summary>
		float SquaredMagnitude() {
			return powf(x, 2) + powf(y, 2);
		}

		///<summary> Changes the vector to have a magnitude of 1. </summary>
		void Normalize() {
			float mag = Magnitude();
			if (mag == 0)
				return;
			x = x / mag;
			y = y / mag;
		}

		///<summary> Returns a new vector with the same direction but a magnitude of 1. </summary>
		pVector Normalized() {
			float mag = Magnitude();
			if (mag == 0)
				return pVector();
			return pVector(x / mag, y / mag);
		}

		///<summary> Checks if the current vector would be inside a box defined by the given values. </summary>
		bool InsideBox(pVector origin, pVector size) {
			return !((x < origin.x || x > origin.x + size.x) || (y < origin.y || y > origin.y + size.y));
		}

		/*
		///<summary> Checks if the current vector would be inside a box defined by the given SDL_Rect. </summary>
		bool InsideBox(SDL_Rect box) {
			return InsideBox(pVector(box.x, box.y), pVector(box.w, box.h));
		}
		*/
	};

	float Distance(pVector p1, pVector p2) {
		return sqrtf(powf(p1.x - p2.x, 2) + powf(p1.y - p2.y, 2));
	}

	float SquaredDistance(pVector p1, pVector p2) {
		return powf(p1.x - p2.x, 2) + powf(p1.y - p2.y, 2);
	}

	float Clamp(float val, float min, float max) {
		if (val < min)
			return min;
		if (val > max)
			return max;
		return val;
	}

	float Dot(pVector v1, pVector v2) {
		return (v1.x * v2.x) + (v1.y * v2.y);
	}

	pVector NearestPointOnFiniteLine(pVector start, pVector end, pVector pnt)
	{
		pVector line = (end - start);
		float len = line.Magnitude();
		line.Normalize();

		pVector v = pnt - start;
		float d = Dot(v, line);
		d = Clamp(d, 0, len);
		return start + line * d;
	}


	///<summary> Represents a physical object with a position, size, velocity and either rectangle or circle shape. </summary>
	class Body
	{
		//All values are public for easier debugging.
	public:
		///<summary> Defines whether the object is a circle or a rectangle. </summary>
		BodyType type = None;
		///<summary> Top-left position of the object. </summary>
		pVector pos;
		///<summary> Radius of the object if it's a circle. </summary>
		float radius;
		///<summary> Size of the object from the top-left extending down and right if it is a box/rectangle. </summary>
		pVector size;
		///<summary> Distance the object will try to move each update. </summary>
		pVector velocity;

		static int SCREEN_WIDTH;
		static int SCREEN_HEIGHT;

		static bool align_collisions;

		static void Initialize(int screenWidth, int screenHeight)
		{
			SCREEN_WIDTH = screenWidth;
			SCREEN_HEIGHT = screenHeight;
			align_collisions = false;
		}

		///<summary> Create a new Body object with a type of 'Box' and the specified starting position and size. </summary>
		Body(pVector Pos, pVector Size) {
			type = Box;
			pos = Pos;
			size = Size;
		}

		///<summary> Create a new Body object with a type of 'Circle' and the specified starting position and radius. </summary>
		Body(pVector Pos, float Radius) {
			type = Circle;
			pos = Pos;
			radius = Radius;
			size = pVector(radius * 2, radius * 2);
		}

		///<summary> Move this body based on its velocity, collide with the list of bodies, and wrap the object based on the screen size. </summary>
		void AutoMoveAndWrap(vector<Body*> allBodies) {
			if (align_collisions)
				MoveAndAlign(velocity, allBodies);
			else
				Move(velocity, allBodies);
			Wrap();
		}

		///<summary> Move this body based on its velocity, and collide with the list of bodies. </summary>
		void AutoMove(vector<Body*> allBodies) {
			if (align_collisions)
				MoveAndAlign(velocity, allBodies);
			else
				Move(velocity, allBodies);
		}

		///<summary> Move this body based on manual motion, and collide with the list of bodies. </summary>
		void Move(pVector Motion, vector<Body*> allBodies) {
			pVector origin = pos;
			pos = pos + Motion;

			for each (Body* other in allBodies)
			{
				if (other == this)
					continue;
				if (CheckCollision(other)) {
					pos = origin;
					break;
				}
			}
		}

		///<summary> [Incomplete] Move based on given value, then align to be 0 pixels away from any collided object. </summary>
		void MoveAndAlign(pVector Motion, vector<Body*> allBodies) {
			pVector origin = pos;
			pos = pos + Motion;

			for each (Body* other in allBodies)
			{
				if (other == this)
					continue;
				if (CheckCollision(other)) {
					//Instead of this, loop, moving by a magnitude of -1 until there isn't a collision.
					pos = origin;
					break;
				}
			}
		}

		///<summary> Ensure that the body is not off screen by teleporting it to the opposite side. </summary>
		void Wrap() {
			if (pos.x > SCREEN_WIDTH)
				pos.x = 0;
			else if (pos.x < 0)
				pos.x = SCREEN_WIDTH;

			if (pos.y > SCREEN_HEIGHT)
				pos.y = 0;
			else if (pos.y < 0)
				pos.y = SCREEN_HEIGHT;
		}

		///<summary> Check for a collision between this and another body (automatically does the correct circle/box collision check). </summary>
		bool CheckCollision(Body* other) {
			if (other->type == type) {
				if (type == Circle)
					return CircleCollidesCircle(other);
				else
					return BoxCollidesBox(other);
			}
			else {
				if (type == Box)
					return CircleCollidesBox(other);
				else
					return other->CircleCollidesBox(this);
			}
		}

		///<summary> Returns true if the distance between the two circles is less than their combined radiuses. </summary>
		bool CircleCollidesCircle(Body* other) {
			return (Distance(pos, other->pos) < radius + other->radius);
		}

		///<summary> Collides *this* circle with a different box body. </summary>
		bool CircleCollidesBox(Body* circle) {
			//No collision if bounding boxes do not intersect.
			if (!BoxCollidesBox(circle->pos - pVector(circle->radius, circle->radius), circle->size)) {
				//circle->sprite.rect.x = 0;
				return false;
			}

			//If circles center is within vertical or horizontal projection, it is colliding (because bounding box collided).
			if (circle->pos.x > pos.x && circle->pos.x < pos.x + size.x)
				return true;
			if (circle->pos.y > pos.y && circle->pos.y < pos.y + size.y)
				return true;

			//Otherwise the only collision can be with one of the corners of the box.
			float SquaredRadius = circle->radius * circle->radius; //Squareroot is very slow.
			if (SquaredDistance(pos, circle->pos) < SquaredRadius
				|| SquaredDistance(pos + pVector(size.x, 0), circle->pos) < SquaredRadius
				|| SquaredDistance(pos + pVector(0, size.y), circle->pos) < SquaredRadius
				|| SquaredDistance(pos + pVector(size.x, size.y), circle->pos) < SquaredRadius) {
				return true;
			}

			return false;
		}

		///<summary> Returns true if either box intersects with the other. </summary>
		bool BoxCollidesBox(Body* other) {
			return ((pos.y < other->pos.y + other->size.y)
				&& (pos.y + size.y > other->pos.y)
				&& (pos.x < other->pos.x + other->size.x)
				&& (pos.x + size.x > other->pos.x));
		}

		///<summary> Returns true if either box intersects with the other. Takes a position and size instead of a body. </summary>
		bool BoxCollidesBox(pVector oPos, pVector oSize) {
			return ((pos.y < oPos.y + oSize.y)
				&& (pos.y + size.y > oPos.y)
				&& (pos.x < oPos.x + oSize.x)
				&& (pos.x + size.x > oPos.x));
		}
	};

	enum BodyType {
		Box,
		Circle,
		None
	};
}