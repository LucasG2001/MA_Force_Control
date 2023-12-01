#ifndef ATOMIC_TASKS_H
#define ATOMIC_TASKS_H

#include "ActionPrimitive.h"

class GetMe : public ActionPrimitive {
public:
	// Additional methods or overrides specific to GetMeTask
	void performAction() override;
};

class FollowMe : public ActionPrimitive {
public:
	// Additional methods or overrides specific to FollowMeTask
	void performAction() override;
};

class HoldThis : public ActionPrimitive {
public:
	// Additional methods or overrides specific to HoldThisTask
	void performAction() override;
};

class TakeThis : public ActionPrimitive {
public:
	// Additional methods or overrides specific to TakeThisTask
	void performAction() override;
};

class AvoidMe : public ActionPrimitive {
public:
	// Additional methods or overrides specific to AvoidMeTask
	void performAction() override;
};

#endif // ATOMIC_TASKS_H
