#include "rosplan_planning_system/PlanParsing/CFFPlanParser.h"

namespace {

    typedef std::vector<std::string> OperatorParams;

    unsigned int split_string(const std::string &text, std::vector<std::string> &tokens, char separator) {
        size_t pos = text.find(separator);
        unsigned int initialPos = 0;
        tokens.clear();

        while(pos != std::string::npos && pos < text.length()) {
            if(text.substr(initialPos, pos - initialPos + 1) !=" ") {
                std::string s = text.substr(initialPos, pos - initialPos + 1);
                s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
                tokens.push_back(s);
            }
            initialPos = pos + 1;
            pos = text.find(separator, initialPos);
        }

        tokens.push_back(text.substr(initialPos, text.size() - initialPos));
        return tokens.size();
    }

    void extractElementsFromLine(const std::string &line, std::string &action_id, std::string &operator_name, OperatorParams &operator_params, std::string& true_son, std::string& false_son) {

        // Example of actions of Contigent-FF:
        //   17||0 --- FIND_OBJECT C1 ITEM_0 --- SON: 18||0
        //   18||0 --- PICKUP_OBJECT C1 ITEM_0 --- SON: 19||0
        //   0||0 --- LOCATE BALL1 WP2 --- TRUESON: 1||0 --- FALSESON: 1||1

        std::vector<std::string> tokens;

        split_string(line, tokens, ' ');
        action_id = tokens[0];
        operator_name = tokens[2];

        int idx = 3;
        while (idx < tokens.size() && tokens[idx] != "---") {
            if (tokens[idx] == "---") break;
            operator_params.push_back(tokens[idx]);
            idx++;
        }
        idx++;
        if (idx == tokens.size() - 2) {
            true_son = false_son = tokens.back();
        } else {
            true_son = tokens[idx + 1];
            false_son = tokens.back();
        }
//        ROS_INFO("%s, %s", true_son.c_str(), false_son.c_str());
        ROS_DEBUG("KCL: (%s) Elements in line: { %s, %s, etc. }", ros::this_node::getName().c_str(), action_id.c_str(), operator_name.c_str());
    }
}


namespace KCL_rosplan {

	CFFPlanParser::CFFPlanParser(ros::NodeHandle& nh)
	{
		node_handle = &nh;

		// fetching problem info for TILs
		std::string kb = "knowledge_base";
		node_handle->getParam("knowledge_base", kb);

		std::stringstream ss;
		ss << "/" << kb << "/domain/operator_details";
		get_operator_details_client = nh.serviceClient<rosplan_knowledge_msgs::GetDomainOperatorDetailsService>(ss.str().c_str());
		ss.str("");

		// publishing parsed plan
		std::string planTopic = "complete_plan";
		node_handle->getParam("plan_topic", planTopic);
		plan_publisher = node_handle->advertise<rosplan_dispatch_msgs::CFFCompletePlan>(planTopic, 1, true);
	}

	CFFPlanParser::~CFFPlanParser()
	{
	}

	void CFFPlanParser::reset() {
        
		action_list.clear();
	}

	void CFFPlanParser::publishPlan() {
        
		ROS_INFO("KCL: (%s) Plan published.", ros::this_node::getName().c_str());    
		ROS_INFO("KCL: (%s) Is plan empty?: %d", ros::this_node::getName().c_str(), action_list.size() == 0);
 		ROS_DEBUG("KCL: (%s) Num actions: %ld", ros::this_node::getName().c_str(), action_list.size());
        
		rosplan_dispatch_msgs::CFFCompletePlan msg;
		msg.plan = action_list;
        msg.edges = edge_list;
		plan_publisher.publish(msg);
	}

	/*----------------------*/
	/* Post processing plan */
	/*----------------------*/

	/**
	 * parses standard PDDL output, generating a list of action messages.
	 */
	void CFFPlanParser::preparePlan() {

		int curr, next;
		std::string line;
		std::istringstream planfile(planner_output);

		size_t planFreeActionID = 0;
        std::vector<std::string> action_ids;
        std::vector<std::pair<std::string, std::string>> children;

		while (std::getline(planfile, line)) {

			if (line.length()<2)
				break;

			// check to see if the line looks like a planned action
			if (line.find("---", 0) == std::string::npos)
				continue;

			// TODO: parse plan
            std::vector<std::string> tokens;
            split_string(line, tokens, ' ');

            std::string action_id;
            std::string operator_name;
            std::string true_son, false_son;
            OperatorParams operator_params;
            extractElementsFromLine(line, action_id, operator_name, operator_params, true_son, false_son);

			rosplan_dispatch_msgs::ActionDispatch msg;

			// action ID
            msg.action_id = action_ids.size();
            action_ids.push_back(action_id);
            msg.name = operator_name;
			planFreeActionID++;

			// check for parameters
            if (true_son == false_son) {
                processPDDLParameters(msg, operator_params);
            } else {
                // 观察动作，知识库中没有operator_detail
                for (const auto& para : operator_params) {
                    diagnostic_msgs::KeyValue kv;
                    kv.key = "not_specified";
                    kv.value = para;
                    msg.parameters.push_back(kv);
                }
            }
            children.push_back(std::make_pair(true_son, false_son));
			action_list.push_back(msg);
		}
        for (int i = 0; i < children.size(); ++i) {
            rosplan_dispatch_msgs::CFFPlanEdge edge;
            edge.source_node = i;
            edge.true_son_id = std::find(action_ids.begin(), action_ids.end(), children[i].first) - action_ids.begin();
            edge.false_son_id = std::find(action_ids.begin(), action_ids.end(), children[i].second) - action_ids.begin();
            edge_list.push_back(edge);
        }
	}



	/**
	 * processes the parameters of a single PDDL action into an ActionDispatch message
	 */
	void CFFPlanParser::processPDDLParameters(rosplan_dispatch_msgs::ActionDispatch &msg, std::vector<std::string> &params) {
        if (params.empty()) return;
		rosplan_knowledge_msgs::GetDomainOperatorDetailsService srv;
		srv.request.name = msg.name;
		if(!get_operator_details_client.call(srv)) {
			ROS_ERROR("KCL: (%s) could not call Knowledge Base for operator details, %s", ros::this_node::getName().c_str(), msg.name.c_str());
		} else {
			std::vector<diagnostic_msgs::KeyValue> opParams = srv.response.op.formula.typed_parameters;
			for(size_t i=0; i<opParams.size(); i++) {
				diagnostic_msgs::KeyValue pair;
				pair.key = opParams[i].key;
				pair.value = params[i];
				msg.parameters.push_back(pair);
			}
		}
	}
} // close namespace

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {

    ros::init(argc,argv,"rosplan_plan_parser");
    ros::NodeHandle nh("~");

    KCL_rosplan::CFFPlanParser pp(nh);

    // subscribe to planner output
    std::string planTopic = "planner_output";
    nh.getParam("planner_topic", planTopic);
    ros::Subscriber plan_sub = nh.subscribe(planTopic, 1, &KCL_rosplan::PlanParser::plannerCallback, dynamic_cast<KCL_rosplan::PlanParser*>(&pp));

    // start the plan parsing services
    ros::ServiceServer service1 = nh.advertiseService("parse_plan", &KCL_rosplan::PlanParser::parsePlan, dynamic_cast<KCL_rosplan::PlanParser*>(&pp));
    ros::ServiceServer service2 = nh.advertiseService("parse_plan_from_file", &KCL_rosplan::PlanParser::parsePlanFromFile, dynamic_cast<KCL_rosplan::PlanParser*>(&pp));

    ROS_INFO("KCL: (%s) Starting a CFFParsingInterface.", ros::this_node::getName().c_str());
    ROS_INFO("KCL: (%s) Ready to receive", ros::this_node::getName().c_str());
    ros::spin();

    return 0;
}
