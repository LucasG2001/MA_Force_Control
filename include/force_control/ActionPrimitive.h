#ifndef ACTION_PRIMITIVE_H
#define ACTION_PRIMITIVE_H

#include <Eigen/Dense>

class ActionPrimitive {
public:
	// Constructor
	ActionPrimitive();

	// Pure virtual function - to be implemented by derived classes
	virtual void performAction() = 0;

	// Setters
	void setStartPose(const Eigen::Matrix<double, 6, 1>& start_pose);
	void setGoalPose(const Eigen::Matrix<double, 6, 1>& goal_pose);
	void setObjectPose(const Eigen::Matrix<double, 6, 1>& object_pose);
	void setGrasp(bool grasp);
	void setParameters(double spring_stiffness, double damping, double inertia,
	                   double repulsion_stiffness, double repulsion_damping);

	// Getters
	Eigen::Matrix<double, 6, 1> getStartPose() const;
	Eigen::Matrix<double, 6, 1> getGoalPose() const;
	Eigen::Matrix<double, 6, 1> getObjectPose() const;
	bool getGrasp() const;
	double getSpringStiffness() const;
	double getDamping() const;
	double getInertia() const;
	double getRepulsionStiffness() const;
	double getRepulsionDamping() const;

protected:
	Eigen::Matrix<double, 6, 1> start_pose_;
	Eigen::Matrix<double, 6, 1> goal_pose_;
	Eigen::Matrix<double, 6, 1> object_pose_;
	bool grasp_;
	double spring_stiffness_;
	double damping_;
	double inertia_;
	double repulsion_stiffness_;
	double repulsion_damping_;
};

#endif // ACTION_PRIMITIVE_H
