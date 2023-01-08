/*
 * This file describes the class used to dispatch a Simple (sequential) PDDL plan.
 */

#ifndef KCL_cff_dispatcher
#define KCL_cff_dispatcher

#include "PlanDispatcher.h"
#include "rosplan_dispatch_msgs/CFFCompletePlan.h"
#include "rosplan_dispatch_msgs/CFFActionFeedback.h"
#include "rosplan_knowledge_msgs/DomainOperator.h"
#include <map>


namespace KCL_rosplan
{

	class CFFPlanDispatcher: public PlanDispatcher
	{
	private:

		// current plan and time plan was recevied
		rosplan_dispatch_msgs::CFFCompletePlan current_plan;

		ros::ServiceClient queryKnowledgeClient;
		ros::ServiceClient queryDomainClient;

		/* current action to dispatch */
		int current_action;

	public:

		/* constructor */
		explicit CFFPlanDispatcher(ros::NodeHandle& nh);
		~CFFPlanDispatcher();

		void planCallback(rosplan_dispatch_msgs::CFFCompletePlan plan);

		void reset() override;

		bool dispatchPlan(double missionStartTime, double planStartTime) override;

		void feedbackCallback(const rosplan_dispatch_msgs::ActionFeedback::ConstPtr& msg) override;


	};
}

#endif
