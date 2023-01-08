/**
 * This file parses Contingent-FF output and generates a list of ActionDispatch messages.
 */
#include "PlanParser.h"

#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/CFFCompletePlan.h"
#include "rosplan_dispatch_msgs/CFFPlanEdge.h"
#include "rosplan_knowledge_msgs/GetDomainOperatorDetailsService.h"
#include "diagnostic_msgs/KeyValue.h"

#include <string>
#include <sstream>

#ifndef KCL_cff_pddl_plan_parser
#define KCL_cff_pddl_plan_parser

namespace KCL_rosplan {

	class CFFPlanParser: public PlanParser
	{
	private:

		ros::ServiceClient get_operator_details_client;
        std::vector<rosplan_dispatch_msgs::CFFPlanEdge> edge_list;

	protected:

		/* post process plan */
		void processPDDLParameters(rosplan_dispatch_msgs::ActionDispatch &msg, std::vector<std::string> &params);

		/* virtual methods */
		void reset();
		void preparePlan();
		void publishPlan();

	public:

		CFFPlanParser(ros::NodeHandle& nh);
		~CFFPlanParser();

		/* ROS interface */
		ros::Publisher plan_publisher;

	};
} // close namespace

#endif
