
#include <Eigen/Dense>
#include <boost/optional>


namespace Planning
{
	/**
	 * Represents the state of the robot at a particular point in time
	 * with a position and a velocity.
	 */
	struct MotionState {
		Eigen::Vector2d pos, vel;

		MotionState();
		MotionState(const Eigen::Vector2d &p, const Eigen::Vector2d &v);

		bool operator==(MotionState &other) {
			return other.pos == pos && other.vel == vel;
		}

		/**
		 * Returns the number of seconds it would take the robot to get
		 * from the receiver to @other using a constant acceleration.
		 */
		float distTo(MotionState &other);


		/**
		 * The below are some functions for using MotionState objects
		 * as control points in bezier curves.  A good tutorial on some
		 * of this stuff is here:
		 * http://devmag.org.za/2011/04/05/bzier-curves-a-tutorial/
		 *
		 * Cubic bezier curves are specified by 4 control points: [P0, P1, P2, P3]
		 * When dealing with MotionState objects, these are: [a.pos, a.vel, -b.vel, b.pos]
		 */

		/**
		 * Evaluates the bezier from @start to @end at @t.
		 * @param t The point to evaluate the bezier at.  It must be between 0 and 1.
		 */
		static MotionState evaluateBezier(MotionState &start, MotionState &end, float t);
		static Vector2d evaluateBezierPosition(MotionState &start, MotionState &end, float t);
		static Vector2d evaluateBezierVelocity(MotionState &start, MotionState &end, float t);
		
		static 
	};


	/**
	 * Represents a motion path as a series of MotionState objects.  Strung
	 * together, these form a series of connected, continuous cubic bezier
	 * curves.
	 */
	class MotionPath {
	public:
		/**
		 * Create an empty MotionPath
		 */
		MotionPath();

		/**
		 * Create a MotionPath starting at the given state
		 */
		MotionPath(MotionState &state);

		/**
		 * Create a MotionPath from a variable number of given states
		 *
		 * Example:
		 *   path = new MotionPath(3, state1, state2, state3);
		 *
		 * @param ptCount The number of points you're passing in
		 * @param ... A series of MotionState objects (not references)
		 */
		MotionPath(int ptCount, ...);

		/**
		 * The length of the path in meters.
		 */
		float length() const;

		/**
		 * The amount of time the path will take to follow
		 */
		float duration() const;

		/**
		 * The number of waypoints in the path.
		 */
		int size() const;

		/**
		 * Finds the MotionState that the path specifies at the given time.
		 * 
		 * @param t The time to evaluate the path at
		 * @return The MotionState at the given time.  Returns boost::none if
		 *         @t is out of bounds.
		 */
		boost::optional<MotionState> evaluate(float t) const;


	private:
		std::vector<MotionState> _points;
	};
}
