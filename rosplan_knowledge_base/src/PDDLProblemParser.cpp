#include "rosplan_knowledge_base/PDDLProblemParser.h"



extern int yyparse();
extern int yydebug;

namespace VAL1_2 {
    extern yyFlexLexer* yfl;
};

/* implementation of PDDLProblemParser.h */
namespace KCL_rosplan {

    /*-----------------------*/
    /* parsing problem */
    /*-----------------------*/

    /**
     * parse the problem file
     */
    VAL1_2::problem* PDDLProblemParser::parseProblem(const std::string ProblemPath) {

        // only parse Problem once
        if(problem_parsed) return problem;
        problem_parsed = true;

        std::string ProblemFileName = (ProblemPath);
        ROS_INFO("KCL: (%s) Parsing Problem File: %s.", ros::this_node::getName().c_str(), ProblemFileName.c_str());

        // save filename for VAL
        std::vector<char> writable(ProblemFileName.begin(), ProblemFileName.end());
        writable.push_back('\0');
        current_filename = &writable[0];

        // parse Problem
        VAL1_2::current_analysis = val_analysis;  // use the same analysis got from the domain
        std::ifstream ProblemFile;
        ProblemFile.open(ProblemFileName.c_str());
        yydebug = 0;

        VAL1_2::yfl = new yyFlexLexer;

        if (ProblemFile.bad()) {
            ROS_ERROR("KCL: (%s) Failed to open problem file.", ros::this_node::getName().c_str());
            line_no = 0;
            VAL1_2::log_error(VAL1_2::E_FATAL,"Failed to open file");
        } else {
            string content, line;
            while (getline(ProblemFile, line)) content += line + "\n";
            preprocessProblem(content);
            std::stringstream ss(content);

            line_no = 1;
            VAL1_2::yfl->switch_streams(&ss, &std::cout);
            yyparse();

            // Problem name
            problem = VAL1_2::current_analysis->the_problem;
            //problem_name = problem->name;

        }
        delete VAL1_2::yfl;
        ProblemFile.close();

        return problem;

    }

    void PDDLProblemParser::preprocessProblem(string &content) {
        const char* keywords[] = {
                "(unknown", "(oneof"
        };
        for (const auto keyword : keywords) {
            while (true) {
                auto idx = content.find(keyword);
                if (idx == std::string::npos) break;
                auto j = idx + 1;
                for (int depth = 1; depth != 0; ++j) {
                    if (content[j] == '(') ++depth;
                    else if (content[j] == ')') --depth;
                }
                content = content.substr(0, idx) + content.substr(j);
            }
        }
    }
} // close namespace
