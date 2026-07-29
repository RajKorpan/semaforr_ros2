/* \mainpage SemaFORR Explanation
 * \brief Explains plans of a robot.
 *
 * \author Raj Korpan.
 *
 * \version SEMAFORR Explanation 1.0
 *
 *
 */

#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <string>
#include <vector>
#include <sstream>
#include <iterator>
#include <map>
#include <algorithm>
#include <cmath>        //for atan2 and M_PI
#include <sys/time.h>
#include <stdexcept>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <functional>
#include <memory>

using namespace std;

class Explanation : public rclcpp::Node
{
private:
	//! We will be publishing to the "plan_explanations" topic
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr plan_explanations_pub_;
	//! We will be publishing to the "plan_explanations_log" topic
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr plan_explanations_log_pub_;
	//! We will be listening to \decision_log, \crowd_density, \plan, and \original_plan topics
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_decision_log_;
	rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_crowd_density_;
	rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_crowd_risk_;
	// Current log
	string current_log;
	// Current crowd density
	nav_msgs::msg::OccupancyGrid current_crowd_density;
	// Current crowd risk
	nav_msgs::msg::OccupancyGrid current_crowd_risk;
	// Current plans
	// nav_msgs::msg::Path current_plan;
	// nav_msgs::msg::Path current_original_plan;
	vector< vector<double> > current_plan;
	vector< vector<double> > current_original_plan;
	// Message received
	bool log_message_received;
	bool density_message_received;
	bool risk_message_received;
	// bool plan_message_received;
	// bool orig_plan_message_received;
	// Stats on plans
	double planDistance=0, originalPlanDistance=0, planCost=0, originalPlanCost=0, planCrowdDensity=0, originalPlanCrowdDensity=0, planCrowdRisk=0, originalPlanCrowdRisk=0;
	// thresholds and their associated phrases
	map <string, vector < pair<double, string> > > thresholds;
	// objectives and their associated phrases
	map <string, vector <string> > objectives;
	// plan directions phrases
	map <int, string> directions_phrases;
	// plan distances phrases
	vector <double> distances_thresholds;
	vector <string> distances_phrases;
	// // distance intervals with their associated phrases
	// vector <double> distanceThreshold;
	// vector <string> distancePhrase;
	// // density intervals with their associated phrases
	// vector <double> densityThreshold;
	// vector <string> densityPhrase;
	// // risk intervals with their associated phrases
	// vector <double> riskThreshold;
	// vector <string> riskPhrase;
	// // flow intervals with their associated phrases
	// vector <double> flowThreshold;
	// vector <string> flowPhrase;
	// // convey intervals with their associated phrases
	// vector <double> conveyThreshold;
	// vector <string> conveyPhrase;
	// // hallway intervals with their associated phrases
	// vector <double> hallwayThreshold;
	// vector <string> hallwayPhrase;
	// // region intervals with their associated phrases
	// vector <double> regionThreshold;
	// vector <string> regionPhrase;
	// // trail intervals with their associated phrases
	// vector <double> trailThreshold;
	// vector <string> trailPhrase;
	// // skeleton intervals with their associated phrases
	// vector <double> skeletonThreshold;
	// vector <string> skeletonPhrase;
	// // highway intervals with their associated phrases
	// vector <double> highwayThreshold;
	// vector <string> highwayPhrase;
	// other parameters
	double targetX, targetY, robotY, robotX;
	string selected_planner;
	vector<string> alternative_planners;
	string alt_planner;
	bool sameplan = 1;
	double computationTimeSec=0.0;
	int densityNum = 5;
	int riskNum = 5;
	int costNum = 5;
	int distanceNum = 5;

public:
	//! ROS node initialization
	Explanation() : rclcpp::Node("why_plan")
	{
		declare_parameter<string>("decision_log_topic", "decision_log");
		declare_parameter<string>("crowd_density_topic", "crowd_density");
		declare_parameter<string>("crowd_risk_topic", "crowd_risk");
		declare_parameter<string>("plan_explanations_topic", "plan_explanations");
		declare_parameter<string>(
			"plan_explanations_log_topic", "plan_explanations_log");
		plan_explanations_pub_ = create_publisher<std_msgs::msg::String>(
			get_parameter("plan_explanations_topic").as_string(), 10);
		plan_explanations_log_pub_ = create_publisher<std_msgs::msg::String>(
			get_parameter("plan_explanations_log_topic").as_string(), 10);
		sub_decision_log_ = create_subscription<std_msgs::msg::String>(
			get_parameter("decision_log_topic").as_string(),
			100,
			bind(&Explanation::updateLog, this, placeholders::_1));
		sub_crowd_density_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
			get_parameter("crowd_density_topic").as_string(),
			10,
			bind(&Explanation::updateCrowdDensity, this, placeholders::_1));
		sub_crowd_risk_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
			get_parameter("crowd_risk_topic").as_string(),
			10,
			bind(&Explanation::updateCrowdRisk, this, placeholders::_1));
		// sub_plan_ = nh.subscribe("plan", 1000, &Explanation::updatePlan, this);
		// sub_original_plan_ = nh.subscribe("original_plan", 1000, &Explanation::updateOriginalPlan, this);
		log_message_received = false;
		density_message_received = false;
		risk_message_received = false;
		// plan_message_received = false;
		// orig_plan_message_received = false;
	}

	void validateConfigGroup(
		const vector<string>& fields,
		size_t group_size,
		const string& key) const
	{
		if(fields.size() <= 1 || (fields.size() - 1) % group_size != 0){
			throw runtime_error(
				"Malformed '" + key + "' entry in plan explanation config");
		}
	}

	bool validPlanField(const string& field)
	{
		const vector<string> segments = parseText(field, ';');
		if(segments.empty()){
			return false;
		}
		for(const string& segment : segments){
			if(parseText(segment, ' ').size() < 2){
				return false;
			}
		}
		return true;
	}

	void updateLog(const std_msgs::msg::String::ConstSharedPtr log){
		const vector<string> fields = parseText(log->data, '\t');
		if(
			fields.size() <= 25 ||
			!validPlanField(fields[16]) ||
			!validPlanField(fields[17]))
		{
			RCLCPP_WARN(
				get_logger(), "Ignoring malformed plan decision diagnostics");
			return;
		}
		if(fields[25].length() > 1){
			log_message_received = true;
			current_log = log->data;
			//RCLCPP_INFO_STREAM(get_logger(), "Recieved log data: " << current_log << endl);
		}
	}

	void updateCrowdDensity(
		const nav_msgs::msg::OccupancyGrid::ConstSharedPtr crowd_density){
		density_message_received = true;
		current_crowd_density = *crowd_density;
		//RCLCPP_INFO_STREAM(get_logger(), "Recieved crowd density data: " << current_crowd_density << endl);
	}

	void updateCrowdRisk(
		const nav_msgs::msg::OccupancyGrid::ConstSharedPtr crowd_risk){
		risk_message_received = true;
		current_crowd_risk = *crowd_risk;
		//RCLCPP_INFO_STREAM(get_logger(), "Recieved crowd density data: " << current_crowd_risk << endl);
	}

	void initialize(string text_config){
		string fileLine;
		ifstream file(text_config.c_str());
		RCLCPP_DEBUG_STREAM(get_logger(), "Reading text_config_file:" << text_config);
		if(!file.is_open()){
			throw runtime_error(
				"Unable to locate or read plan explanation text config: " +
				text_config);
		}

		while(getline(file, fileLine)){
			//cout << "Inside while in tasks" << endl;
			if(fileLine.empty() || fileLine[0] == '#'){  // skip comment lines
				continue;
			}
			else if (fileLine.find("threshold") != string::npos){
				vector<string> vstrings = parseText(fileLine, '\t');
				validateConfigGroup(vstrings, 7, "threshold");
				RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[0]);
				for(size_t i=1; i < vstrings.size(); i+=7){
					vector < pair<double, string> > thresh;
					thresh.push_back(pair<double, string>(atof(vstrings[i+1].c_str()),vstrings[i+2]));
					thresh.push_back(pair<double, string>(atof(vstrings[i+3].c_str()),vstrings[i+4]));
					thresh.push_back(pair<double, string>(atof(vstrings[i+5].c_str()),vstrings[i+6]));
					thresholds.insert(pair< string, vector < pair<double, string> > >(vstrings[i], thresh));
					RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[i] << " " << vstrings[i+1] << " " << vstrings[i+2] << " " << vstrings[i+3] << " " << vstrings[i+4] << " " << vstrings[i+5] << " " << vstrings[i+6] << endl);
				}
			}
			else if (fileLine.find("objphrases") != string::npos){
				vector<string> vstrings = parseText(fileLine, '\t');
				validateConfigGroup(vstrings, 4, "objphrases");
				RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[0]);
				for(size_t i=1; i < vstrings.size(); i+=4){
					vector <string> objs;
					objs.push_back(vstrings[i+1]);
					objs.push_back(vstrings[i+2]);
					objs.push_back(vstrings[i+3]);
					objectives.insert(pair< string, vector <string> >(vstrings[i], objs));
					RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[i] << " " << vstrings[i+1] << " " << vstrings[i+2] << " " << vstrings[i+3] << endl);
				}
			}
			else if (fileLine.find("plandirections") != string::npos){
				vector<string> vstrings = parseText(fileLine, '\t');
				validateConfigGroup(vstrings, 2, "plandirections");
				RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[0]);
				for(size_t i=1; i < vstrings.size(); i+=2){
					directions_phrases.insert(pair< int, string >(atoi(vstrings[i].c_str()), vstrings[i+1]));
					RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[i] << " " << vstrings[i+1] << endl);
				}
			}
			else if (fileLine.find("plandistances") != string::npos){
				vector<string> vstrings = parseText(fileLine, '\t');
				validateConfigGroup(vstrings, 2, "plandistances");
				RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[0]);
				for(size_t i=1; i < vstrings.size(); i+=2){
					distances_thresholds.push_back(atof(vstrings[i].c_str()));
					distances_phrases.push_back(vstrings[i+1]);
					RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[i] << " " << vstrings[i+1] << endl);
				}
			}
			// else if (fileLine.find("distance") != string::npos){
			// 	vector<string> vstrings = parseText(fileLine, '\t');
			// 	for(size_t i=1; i < vstrings.size(); i+=2){
			// 		distanceThreshold.push_back(atof(vstrings[i].c_str()));
			// 		distancePhrase.push_back(vstrings[i+1]);
			// 		//RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[i+1] << " " << vstrings[i] << endl);
			// 	}
			// 	//RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[0]);
			// }
			// else if (fileLine.find("density") != string::npos){
			// 	vector<string> vstrings = parseText(fileLine, '\t');
			// 	for(size_t i=1; i < vstrings.size(); i+=2){
			// 		densityThreshold.push_back(atof(vstrings[i].c_str()));
			// 		densityPhrase.push_back(vstrings[i+1]);
			// 		//RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[i+1] << " " << vstrings[i] << endl);
			// 	}
			// 	//RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[0]);
			// }
			// else if (fileLine.find("risk") != string::npos){
			// 	vector<string> vstrings = parseText(fileLine, '\t');
			// 	for(size_t i=1; i < vstrings.size(); i+=2){
			// 		riskThreshold.push_back(atof(vstrings[i].c_str()));
			// 		riskPhrase.push_back(vstrings[i+1]);
			// 		//RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[i+1] << " " << vstrings[i] << endl);
			// 	}
			// 	//RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[0]);
			// }
			// else if (fileLine.find("flow") != string::npos){
			// 	vector<string> vstrings = parseText(fileLine, '\t');
			// 	for(size_t i=1; i < vstrings.size(); i+=2){
			// 		flowThreshold.push_back(atof(vstrings[i].c_str()));
			// 		flowPhrase.push_back(vstrings[i+1]);
			// 		//RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[i+1] << " " << vstrings[i] << endl);
			// 	}
			// 	//RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[0]);
			// }
			// else if (fileLine.find("conveys") != string::npos){
			// 	vector<string> vstrings = parseText(fileLine, '\t');
			// 	for(size_t i=1; i < vstrings.size(); i+=2){
			// 		conveyThreshold.push_back(atof(vstrings[i].c_str()));
			// 		conveyPhrase.push_back(vstrings[i+1]);
			// 		//RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[i+1] << " " << vstrings[i] << endl);
			// 	}
			// 	//RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[0]);
			// }
			// else if (fileLine.find("hallwayer") != string::npos){
			// 	vector<string> vstrings = parseText(fileLine, '\t');
			// 	for(size_t i=1; i < vstrings.size(); i+=2){
			// 		hallwayThreshold.push_back(atof(vstrings[i].c_str()));
			// 		hallwayPhrase.push_back(vstrings[i+1]);
			// 		//RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[i+1] << " " << vstrings[i] << endl);
			// 	}
			// 	//RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[0]);
			// }
			// else if (fileLine.find("spatial") != string::npos){
			// 	vector<string> vstrings = parseText(fileLine, '\t');
			// 	for(size_t i=1; i < vstrings.size(); i+=2){
			// 		regionThreshold.push_back(atof(vstrings[i].c_str()));
			// 		regionPhrase.push_back(vstrings[i+1]);
			// 		//RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[i+1] << " " << vstrings[i] << endl);
			// 	}
			// 	//RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[0]);
			// }
			// else if (fileLine.find("trailer") != string::npos){
			// 	vector<string> vstrings = parseText(fileLine, '\t');
			// 	for(size_t i=1; i < vstrings.size(); i+=2){
			// 		trailThreshold.push_back(atof(vstrings[i].c_str()));
			// 		trailPhrase.push_back(vstrings[i+1]);
			// 		//RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[i+1] << " " << vstrings[i] << endl);
			// 	}
			// 	//RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[0]);
			// }
			// else if (fileLine.find("skeleton") != string::npos){
			// 	vector<string> vstrings = parseText(fileLine, '\t');
			// 	for(size_t i=1; i < vstrings.size(); i+=2){
			// 		skeletonThreshold.push_back(atof(vstrings[i].c_str()));
			// 		skeletonPhrase.push_back(vstrings[i+1]);
			// 		//RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[i+1] << " " << vstrings[i] << endl);
			// 	}
			// 	//RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[0]);
			// }
			// else if (fileLine.find("hallwayskel") != string::npos){
			// 	vector<string> vstrings = parseText(fileLine, '\t');
			// 	for(size_t i=1; i < vstrings.size(); i+=2){
			// 		highwayThreshold.push_back(atof(vstrings[i].c_str()));
			// 		highwayPhrase.push_back(vstrings[i+1]);
			// 		//RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[i+1] << " " << vstrings[i] << endl);
			// 	}
			// 	//RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[0]);
			// }
		}
		if(
			thresholds.empty() || objectives.empty() ||
			directions_phrases.empty() || distances_thresholds.empty() ||
			distances_thresholds.size() != distances_phrases.size())
		{
			throw runtime_error(
				"Plan explanation config is missing required entries");
		}
		rclcpp::spin_some(shared_from_this());
	}
	
	void run(){
		std_msgs::msg::String explanationString;
		rclcpp::Rate rate(30.0);
		timeval cv;
		double start_timecv, end_timecv;
		while(rclcpp::ok()) {
			//while(log_message_received == false or plan_message_received == false or orig_plan_message_received == false or (density_message_received == false and risk_message_received == false)){
			while(rclcpp::ok() && log_message_received == false){
				//RCLCPP_DEBUG(get_logger(), "Waiting for all messages");
				//wait for some time
				rate.sleep();
				// Sense input 
				rclcpp::spin_some(shared_from_this());
			}
			if(!rclcpp::ok()){
				break;
			}
			RCLCPP_INFO_STREAM(get_logger(), "Messages received");
			gettimeofday(&cv,NULL);
			start_timecv = cv.tv_sec + (cv.tv_usec/1000000.0);
			vector<string> parsed_log = parseText(current_log, '\t');
			cout << "current decision " << parsed_log[0] << " " << parsed_log[1] << endl;
			targetX = atof(parsed_log[4].c_str());
			targetY = atof(parsed_log[5].c_str());
			robotX = atof(parsed_log[6].c_str());
			robotY = atof(parsed_log[7].c_str());
			cout << "planners " << parsed_log[25] << endl;
			selected_planner = parseText(parsed_log[25], '>')[0];
			alternative_planners = parseText(parsed_log[25], '>');
			for(size_t i = 0; i < alternative_planners.size(); i++){
				cout << alternative_planners[i] << endl;
			}
			alternative_planners.erase(alternative_planners.begin());
			alt_planner = alternative_planners.empty()
				? "distance"
				: alternative_planners.front();
			savePlanCosts(parsed_log);
			if(density_message_received){
				computePlanDensities();
			}
			if(risk_message_received){
				computePlanRisks();
			}
			if(selected_planner != "hallwayskel" and selected_planner != "skeletonhall"){
				alt_planner = "distance";
			}
			if(selected_planner == "skeletonhall" or selected_planner == "hallwayskel"){
				planCost = planCost * 100.0;
				originalPlanCost = originalPlanCost * 100.0;
			}
			cout << "selected_planner " << selected_planner << " alt_planner " << alt_planner << endl;
			//RCLCPP_INFO_STREAM(get_logger(), "Before compute plan distances");
			//computePlanDistances();
			//RCLCPP_INFO_STREAM(get_logger(), "Before compute plan densities");
			//computePlanDensities();
			//computePlanRisks();
			//RCLCPP_INFO_STREAM(get_logger(), "Before compare plans");
			if(selected_planner == "distance"){
				explanationString.data = "I decided to go this way because I agree that we should take the shortest route.\nActually, I agree that we should take the shortest route.\nYour way is the best way to go.\nI'm really sure because this is the shortest way.";
			}
			else if (comparePlans()) {
				explanationString.data = "I decided to go this way because I think it is just as " + objectivePhrase(alt_planner, 0) + " and equally " + objectivePhrase(selected_planner, 0) + ".\nI think both plans are equally good.\nWe could go your way since it's a bit " + objectivePhrase(alt_planner, 1) + " but it could also be a bit " + objectivePhrase(selected_planner, 2) + ".\nI'm only somewhat sure because even though my plan is a bit " + objectivePhrase(selected_planner, 1) + ", it is also a bit " + objectivePhrase(alt_planner, 2) + " than your plan.";
			}
			else if ((planDistance - originalPlanDistance) <= 0) {
				explanationString.data = "I think my way is " + costDifftoPhrase(selected_planner) + " " + objectivePhrase(selected_planner, 1) + ".\n" + "I prefer my plan because it's " + costDifftoPhrase(selected_planner) + " " + objectivePhrase(selected_planner, 1) + ".\n" + "We could go your way since it's a bit " + objectivePhrase(alt_planner, 1) + " but it could also be " + costDifftoPhrase(selected_planner) + " " + objectivePhrase(selected_planner, 2) + ".\n" + "I'm " + computeCostConf() + ".";
			}
			else {
				explanationString.data = "Although there may be another way that is " + distanceDiffToPhrase(alt_planner) + " " + objectivePhrase(alt_planner, 1) + ", I think my way is " + costDifftoPhrase(selected_planner) + " " + objectivePhrase(selected_planner, 1) + ".\n" + "I prefer my plan because it's " + costDifftoPhrase(selected_planner) + " " + objectivePhrase(selected_planner, 1) + ".\n" + "We could go your way since it's " + distanceDiffToPhrase(alt_planner) + " " + objectivePhrase(alt_planner, 1) + " but it could also be " + costDifftoPhrase(selected_planner) + " " + objectivePhrase(selected_planner, 2) + ".\n" + "I'm " + computeCostConf() + ".";
			}
			// give explanation for them
			cout << "Give plan step based explanation" << endl;
			if(current_plan.size() > 2){
				vector<double> current_plan_angles;
				vector<double> current_plan_distances;
				for(size_t i = 0; i < current_plan.size()-1; i++){
					current_plan_angles.push_back(atan2(current_plan[i][1] - current_plan[i+1][1], current_plan[i][0] - current_plan[i+1][0]));
					current_plan_distances.push_back(computeDistance(current_plan[i][0], current_plan[i][1], current_plan[i+1][0], current_plan[i+1][1]));
					cout << computeDistance(current_plan[i][0], current_plan[i][1], current_plan[i+1][0], current_plan[i+1][1]) << endl;
				}
				cout << "current plan angles " << current_plan_angles.size() << " current plan distances " << current_plan_distances.size() << endl;
				vector<int> current_plan_directions;
				for(size_t i = 0; i < current_plan_angles.size(); i++){
					if(current_plan_angles[i] >= -M_PI/8.0 and current_plan_angles[i] < M_PI/8.0){
						current_plan_directions.push_back(5);
					}
					else if(current_plan_angles[i] >= M_PI/8.0 and current_plan_angles[i] < 3.0*M_PI/8.0){
						current_plan_directions.push_back(6);
					}
					else if(current_plan_angles[i] >= 3.0*M_PI/8.0 and current_plan_angles[i] < 5.0*M_PI/8.0){
						current_plan_directions.push_back(7);
					}
					else if(current_plan_angles[i] >= 5.0*M_PI/8.0 and current_plan_angles[i] < 7.0*M_PI/8.0){
						current_plan_directions.push_back(8);
					}
					else if(current_plan_angles[i] >= 7.0*M_PI/8.0 or current_plan_angles[i] < -7.0*M_PI/8.0){
						current_plan_directions.push_back(1);
					}
					else if(current_plan_angles[i] >= -7.0*M_PI/8.0 and current_plan_angles[i] < -5.0*M_PI/8.0){
						current_plan_directions.push_back(2);
					}
					else if(current_plan_angles[i] >= -5.0*M_PI/8.0 and current_plan_angles[i] < -3.0*M_PI/8.0){
						current_plan_directions.push_back(3);
					}
					else if(current_plan_angles[i] >= -3.0*M_PI/8.0 and current_plan_angles[i] < -M_PI/8.0){
						current_plan_directions.push_back(4);
					}
				}
				cout << "current plan directions " << current_plan_directions.size() << endl;
				vector<int> current_plan_direction_changes;
				for(size_t i = 0; i < current_plan_directions.size()-1; i++){
					if(current_plan_directions[i+1] - current_plan_directions[i] < 0){
						current_plan_direction_changes.push_back(current_plan_directions[i+1] - current_plan_directions[i] + 8);
					}
					else{
						current_plan_direction_changes.push_back(current_plan_directions[i+1] - current_plan_directions[i]);
					}
				}
				cout << "current plan direction changes " << current_plan_direction_changes.size() << endl;
				vector<string> current_plan_direction_phrases;
				for(size_t i = 0; i < current_plan_direction_changes.size(); i++){
					current_plan_direction_phrases.push_back(directions_phrases[current_plan_direction_changes[i]]);
					cout << directions_phrases[current_plan_direction_changes[i]] << endl;
				}
				cout << "current_plan_direction_phrases " << current_plan_direction_phrases.size() << endl;
				vector<string> current_plan_short_description;
				vector<double> current_plan_segment_distances;
				double seg_dist = 0;
				int start_seg = 0;
				int end_seg = 0;
				for(size_t i = 0; i < current_plan_direction_phrases.size()-1; i++){
					cout << "start_seg " << start_seg << " end_seg " << end_seg << " seg_dist " << seg_dist << endl;
					cout << "Phrase " << i << " " << current_plan_direction_phrases[i] << " Phrase " << i+1 << " " << current_plan_direction_phrases[i+1] << endl;
					if(current_plan_direction_phrases[i] == current_plan_direction_phrases[i+1] and current_plan_direction_phrases[i] == directions_phrases[0]){
						end_seg = i + 1;
					}
					else if(current_plan_direction_phrases[i] != current_plan_direction_phrases[i+1] and current_plan_direction_phrases[i] == directions_phrases[0]){
						current_plan_short_description.push_back(current_plan_direction_phrases[i]);
						end_seg = i + 1;
						for(int j = start_seg; j <= end_seg; j++){
							seg_dist = seg_dist + current_plan_distances[j];
						}
						cout << "seg_dist " << seg_dist << endl;
						current_plan_segment_distances.push_back(seg_dist);
						start_seg = end_seg + 1;
						seg_dist = 0;
					}
					else if(current_plan_direction_phrases[i] != directions_phrases[0] and current_plan_direction_phrases[i+1] == directions_phrases[0]){
						current_plan_short_description.push_back(current_plan_direction_phrases[i]);
						seg_dist = 0;
						cout << "seg_dist " << seg_dist << endl;
						current_plan_segment_distances.push_back(seg_dist);
					}
					else if(current_plan_direction_phrases[i] != directions_phrases[0] and current_plan_direction_phrases[i+1] != directions_phrases[0]){
						current_plan_short_description.push_back(current_plan_direction_phrases[i]);
						seg_dist = 0;
						cout << "seg_dist " << seg_dist << endl;
						current_plan_segment_distances.push_back(seg_dist);
						current_plan_short_description.push_back(directions_phrases[0]);
						seg_dist = current_plan_distances[i+1];
						cout << "seg_dist " << seg_dist << endl;
						current_plan_segment_distances.push_back(seg_dist);
						start_seg = start_seg + 1;
						end_seg = i + 1;
					}
					cout << "start_seg " << start_seg << " end_seg " << end_seg << " seg_dist " << seg_dist << endl;
				}
				if(current_plan_direction_phrases[current_plan_direction_phrases.size()-1] == directions_phrases[0]){
					current_plan_short_description.push_back(current_plan_direction_phrases[current_plan_direction_phrases.size()-1]);
					end_seg = end_seg + 1;
					for(int j = start_seg; j <= end_seg; j++){
						seg_dist = seg_dist + current_plan_distances[j];
					}
					cout << "seg_dist " << seg_dist << endl;
					current_plan_segment_distances.push_back(seg_dist);
				}
				else{
					current_plan_short_description.push_back(current_plan_direction_phrases[current_plan_direction_phrases.size()-1]);
					seg_dist = 0;
					cout << "seg_dist " << seg_dist << endl;
					current_plan_segment_distances.push_back(seg_dist);
					current_plan_short_description.push_back(directions_phrases[0]);
					seg_dist = current_plan_distances[current_plan_direction_phrases.size()];
					cout << "seg_dist " << seg_dist << endl;
					current_plan_segment_distances.push_back(seg_dist);
				}
				if(current_plan_short_description[0] != directions_phrases[0]){
					current_plan_short_description.insert(current_plan_short_description.begin(), directions_phrases[0]);
					current_plan_segment_distances.insert(current_plan_segment_distances.begin(), current_plan_distances[0]);
				}
				cout << "current_plan_short_description " << current_plan_short_description.size() << endl;
				string plan_description = "We will ";
				for(size_t i = 0; i < current_plan_short_description.size()-1; i++){
					cout << current_plan_short_description[i] << " " << current_plan_segment_distances[i] << endl;
					if(current_plan_segment_distances[i] == 0){
						if(selected_planner == "hallwayskel"){
							plan_description = plan_description + current_plan_short_description[i] + " at an intersection, ";
						}
						else{
							plan_description = plan_description + current_plan_short_description[i] + ", ";
						}
					}
					else{
						string dist_phrase;
						for(int j = distances_thresholds.size()-1; j >= 0; --j) {
							if(current_plan_segment_distances[i] <= distances_thresholds[j]) {
								dist_phrase = distances_phrases[j];
							}
						}
						RCLCPP_INFO_STREAM(get_logger(), current_plan_segment_distances[i] << " " << dist_phrase);
						plan_description = plan_description + current_plan_short_description[i] + " about " + dist_phrase + ", ";
					}
				}
				cout << current_plan_short_description[current_plan_short_description.size()-1] << " " << current_plan_segment_distances[current_plan_short_description.size()-1] << endl;
				if(current_plan_segment_distances[current_plan_short_description.size()-1] == 0){
					plan_description = plan_description + "and " + current_plan_short_description[current_plan_short_description.size()-1] + " to reach our target.";
				}
				else{
					string dist_phrase;
					for(int j = distances_thresholds.size()-1; j >= 0; --j) {
						if(current_plan_segment_distances[current_plan_short_description.size()-1] <= distances_thresholds[j]) {
							dist_phrase = distances_phrases[j];
						}
					}
					RCLCPP_INFO_STREAM(get_logger(), current_plan_segment_distances[current_plan_short_description.size()-1] << " " << dist_phrase);
					plan_description = plan_description + "and " + current_plan_short_description[current_plan_short_description.size()-1] + " about " + dist_phrase + " to reach our target.";
				}
				cout << plan_description << endl;
				explanationString.data = explanationString.data + "\n" + plan_description;
			}
			else if(current_plan.size() == 2){
				explanationString.data = explanationString.data + "\n" + "We will go directly to our target.";
			}
			if(current_original_plan.size() > 2){
				vector<double> alt_plan_angles;
				vector<double> alt_plan_distances;
				for(size_t i = 0; i < current_original_plan.size()-1; i++){
					alt_plan_angles.push_back(atan2(current_original_plan[i][1] - current_original_plan[i+1][1], current_original_plan[i][0] - current_original_plan[i+1][0]));
					alt_plan_distances.push_back(computeDistance(current_original_plan[i][0], current_original_plan[i][1], current_original_plan[i+1][0], current_original_plan[i+1][1]));
					cout << computeDistance(current_original_plan[i][0], current_original_plan[i][1], current_original_plan[i+1][0], current_original_plan[i+1][1]) << endl;
				}
				cout << "alt plan angles " << alt_plan_angles.size() << " alt plan distances " << alt_plan_distances.size() << endl;
				vector<int> alt_plan_directions;
				for(size_t i = 0; i < alt_plan_angles.size(); i++){
					if(alt_plan_angles[i] >= -M_PI/8.0 and alt_plan_angles[i] < M_PI/8.0){
						alt_plan_directions.push_back(5);
					}
					else if(alt_plan_angles[i] >= M_PI/8.0 and alt_plan_angles[i] < 3.0*M_PI/8.0){
						alt_plan_directions.push_back(6);
					}
					else if(alt_plan_angles[i] >= 3.0*M_PI/8.0 and alt_plan_angles[i] < 5.0*M_PI/8.0){
						alt_plan_directions.push_back(7);
					}
					else if(alt_plan_angles[i] >= 5.0*M_PI/8.0 and alt_plan_angles[i] < 7.0*M_PI/8.0){
						alt_plan_directions.push_back(8);
					}
					else if(alt_plan_angles[i] >= 7.0*M_PI/8.0 or alt_plan_angles[i] < -7.0*M_PI/8.0){
						alt_plan_directions.push_back(1);
					}
					else if(alt_plan_angles[i] >= -7.0*M_PI/8.0 and alt_plan_angles[i] < -5.0*M_PI/8.0){
						alt_plan_directions.push_back(2);
					}
					else if(alt_plan_angles[i] >= -5.0*M_PI/8.0 and alt_plan_angles[i] < -3.0*M_PI/8.0){
						alt_plan_directions.push_back(3);
					}
					else if(alt_plan_angles[i] >= -3.0*M_PI/8.0 and alt_plan_angles[i] < -M_PI/8.0){
						alt_plan_directions.push_back(4);
					}
				}
				cout << "alt plan directions " << alt_plan_directions.size() << endl;
				vector<int> alt_plan_direction_changes;
				for(size_t i = 0; i < alt_plan_directions.size()-1; i++){
					if(alt_plan_directions[i+1] - alt_plan_directions[i] < 0){
						alt_plan_direction_changes.push_back(alt_plan_directions[i+1] - alt_plan_directions[i] + 8);
					}
					else{
						alt_plan_direction_changes.push_back(alt_plan_directions[i+1] - alt_plan_directions[i]);
					}
				}
				cout << "alt plan direction changes " << alt_plan_direction_changes.size() << endl;
				vector<string> alt_plan_direction_phrases;
				for(size_t i = 0; i < alt_plan_direction_changes.size(); i++){
					alt_plan_direction_phrases.push_back(directions_phrases[alt_plan_direction_changes[i]]);
					cout << directions_phrases[alt_plan_direction_changes[i]] << endl;
				}
				cout << "alt_plan_direction_phrases " << alt_plan_direction_phrases.size() << endl;
				vector<string> alt_plan_short_description;
				vector<double> alt_plan_segment_distances;
				double seg_dist = 0;
				int start_seg = 0;
				int end_seg = 0;
				for(size_t i = 0; i < alt_plan_direction_phrases.size()-1; i++){
					cout << "start_seg " << start_seg << " end_seg " << end_seg << " seg_dist " << seg_dist << endl;
					cout << "Phrase " << i << " " << alt_plan_direction_phrases[i] << " Phrase " << i+1 << " " << alt_plan_direction_phrases[i+1] << endl;
					if(alt_plan_direction_phrases[i] == alt_plan_direction_phrases[i+1] and alt_plan_direction_phrases[i] == directions_phrases[0]){
						end_seg = i + 1;
					}
					else if(alt_plan_direction_phrases[i] != alt_plan_direction_phrases[i+1] and alt_plan_direction_phrases[i] == directions_phrases[0]){
						alt_plan_short_description.push_back(alt_plan_direction_phrases[i]);
						end_seg = i + 1;
						for(int j = start_seg; j <= end_seg; j++){
							seg_dist = seg_dist + alt_plan_distances[j];
						}
						cout << "seg_dist " << seg_dist << endl;
						alt_plan_segment_distances.push_back(seg_dist);
						start_seg = end_seg + 1;
						seg_dist = 0;
					}
					else if(alt_plan_direction_phrases[i] != directions_phrases[0] and alt_plan_direction_phrases[i+1] == directions_phrases[0]){
						alt_plan_short_description.push_back(alt_plan_direction_phrases[i]);
						seg_dist = 0;
						cout << "seg_dist " << seg_dist << endl;
						alt_plan_segment_distances.push_back(seg_dist);
					}
					else if(alt_plan_direction_phrases[i] != directions_phrases[0] and alt_plan_direction_phrases[i+1] != directions_phrases[0]){
						alt_plan_short_description.push_back(alt_plan_direction_phrases[i]);
						seg_dist = 0;
						cout << "seg_dist " << seg_dist << endl;
						alt_plan_segment_distances.push_back(seg_dist);
						alt_plan_short_description.push_back(directions_phrases[0]);
						seg_dist = alt_plan_distances[i+1];
						cout << "seg_dist " << seg_dist << endl;
						alt_plan_segment_distances.push_back(seg_dist);
						start_seg = start_seg + 1;
						end_seg = i + 1;
					}
					cout << "start_seg " << start_seg << " end_seg " << end_seg << " seg_dist " << seg_dist << endl;
				}
				if(alt_plan_direction_phrases[alt_plan_direction_phrases.size()-1] == directions_phrases[0]){
					alt_plan_short_description.push_back(alt_plan_direction_phrases[alt_plan_direction_phrases.size()-1]);
					end_seg = end_seg + 1;
					for(int j = start_seg; j <= end_seg; j++){
						seg_dist = seg_dist + alt_plan_distances[j];
					}
					cout << "seg_dist " << seg_dist << endl;
					alt_plan_segment_distances.push_back(seg_dist);
				}
				else{
					alt_plan_short_description.push_back(alt_plan_direction_phrases[alt_plan_direction_phrases.size()-1]);
					seg_dist = 0;
					cout << "seg_dist " << seg_dist << endl;
					alt_plan_segment_distances.push_back(seg_dist);
					alt_plan_short_description.push_back(directions_phrases[0]);
					seg_dist = alt_plan_distances[alt_plan_direction_phrases.size()];
					cout << "seg_dist " << seg_dist << endl;
					alt_plan_segment_distances.push_back(seg_dist);
				}
				cout << "alt_plan_short_description " << alt_plan_short_description.size() << endl;
				string plan_description = "We could ";
				for(size_t i = 0; i < alt_plan_short_description.size()-1; i++){
					cout << alt_plan_short_description[i] << " " << alt_plan_segment_distances[i] << endl;
					if(alt_plan_segment_distances[i] == 0){
						if(alt_planner == "hallwayskel"){
							plan_description = plan_description + alt_plan_short_description[i] + " at an intersection, ";
						}
						else{
							plan_description = plan_description + alt_plan_short_description[i] + ", ";
						}
					}
					else{
						string dist_phrase;
						for(int j = distances_thresholds.size()-1; j >= 0; --j) {
							if(alt_plan_segment_distances[i] <= distances_thresholds[j]) {
								dist_phrase = distances_phrases[j];
							}
						}
						RCLCPP_INFO_STREAM(get_logger(), alt_plan_segment_distances[i] << " " << dist_phrase);
						plan_description = plan_description + alt_plan_short_description[i] + " about " + dist_phrase + ", ";
					}
				}
				cout << alt_plan_short_description[alt_plan_short_description.size()-1] << " " << alt_plan_segment_distances[alt_plan_short_description.size()-1] << endl;
				if(alt_plan_segment_distances[alt_plan_short_description.size()-1] == 0){
					plan_description = plan_description + "and " + alt_plan_short_description[alt_plan_short_description.size()-1] + " to reach our target.";
				}
				else{
					string dist_phrase;
					for(int j = distances_thresholds.size()-1; j >= 0; --j) {
						if(alt_plan_segment_distances[alt_plan_short_description.size()-1] <= distances_thresholds[j]) {
							dist_phrase = distances_phrases[j];
						}
					}
					RCLCPP_INFO_STREAM(get_logger(), alt_plan_segment_distances[alt_plan_short_description.size()-1] << " " << dist_phrase);
					plan_description = plan_description + "and " + alt_plan_short_description[alt_plan_short_description.size()-1] + " about " + dist_phrase + " to reach our target.";
				}
				cout << plan_description << endl;
				explanationString.data = explanationString.data + "\n" + plan_description;
			}
			else if(current_original_plan.size() == 2){
				explanationString.data = explanationString.data + "\n" + "We could go directly to our target.";
			}
			gettimeofday(&cv,NULL);
			end_timecv = cv.tv_sec + (cv.tv_usec/1000000.0);
			computationTimeSec = (end_timecv-start_timecv);
			RCLCPP_INFO_STREAM(get_logger(), "After Plan explanation: " << explanationString.data);
			//RCLCPP_INFO_STREAM(get_logger(), "Before log data");
			logExplanationData();
			//send the explanation
			plan_explanations_pub_->publish(explanationString);
			log_message_received = false;
			density_message_received = false;
			risk_message_received = false;
			// plan_message_received = false;
			// orig_plan_message_received = false;
			//RCLCPP_INFO_STREAM(get_logger(), "Before clear stats");
			clearStats();
			//RCLCPP_INFO_STREAM(get_logger(), "Loop completed");
			//wait for some time
			rate.sleep();
			// Sense input 
			rclcpp::spin_some(shared_from_this());
		}
	}

	// void computePlanDistances(){
	// 	//RCLCPP_INFO_STREAM(get_logger(), "Inside compute plan distances");
	// 	//planDistance = computeDistance(robotX, robotY, current_plan[0][0], current_plan[0][1]);
	// 	//RCLCPP_INFO_STREAM(get_logger(), "Initial plan distance: " << planDistance);
	// 	for(size_t i = 0; i < current_plan.size()-1; i++){
	// 		planDistance += computeDistance(current_plan[i][0], current_plan[i][1], current_plan[i+1][0], current_plan[i+1][1]);
	// 	}
	// 	//RCLCPP_INFO_STREAM(get_logger(), "Plan distance after loop: " << planDistance);
	// 	planDistance += computeDistance(current_plan[current_plan.size()-1][0], current_plan[current_plan.size()-1][1], targetX, targetY);
	// 	//RCLCPP_INFO_STREAM(get_logger(), "Final plan distance: " << planDistance);

	// 	//originalPlanDistance = computeDistance(robotX, robotY, current_original_plan[0][0], current_original_plan[0][1]);
	// 	//RCLCPP_INFO_STREAM(get_logger(), "Initial orig plan distance: " << originalPlanDistance);
	// 	for(size_t i = 0; i < current_original_plan.size()-1; i++){
	// 		originalPlanDistance += computeDistance(current_original_plan[i][0], current_original_plan[i][1], current_original_plan[i+1][0], current_original_plan[i+1][1]);
	// 	}
	// 	//RCLCPP_INFO_STREAM(get_logger(), "Orig Plan distance after loop: " << originalPlanDistance);
	// 	originalPlanDistance += computeDistance(current_original_plan[current_original_plan.size()-1][0], current_original_plan[current_original_plan.size()-1][1], targetX, targetY);
	// 	//RCLCPP_INFO_STREAM(get_logger(), "Final orig plan distance: " << originalPlanDistance);
	// }

	void savePlanCosts(vector<string> parsed_log){
		RCLCPP_INFO_STREAM(get_logger(), "Inside save plan costs");
		cout << "Plan text " << parsed_log[16] << endl;
		vector<string> vstrings1 = parseText(parsed_log[16], ';');
		planCost = atof(parseText(vstrings1[0], ' ')[0].c_str()) / 100.0; // cost is for selected plan
		planDistance = atof(parseText(vstrings1[0], ' ')[1].c_str()) / 100.0; // distance is for alternative plan
		RCLCPP_INFO_STREAM(get_logger(), "Plan cost: " << planCost << " Plan distance = " << planDistance);
		vector<double> robot_point;
		robot_point.push_back(robotX);
		robot_point.push_back(robotY);
		vector<double> target_point;
		target_point.push_back(targetX);
		target_point.push_back(targetY);
		current_plan.push_back(robot_point);
		for(size_t i = 1; i < vstrings1.size(); i++){
			vector<double> point;
			point.push_back(atof(parseText(vstrings1[i], ' ')[0].c_str()));
			point.push_back(atof(parseText(vstrings1[i], ' ')[1].c_str()));
			current_plan.push_back(point);
		}
		current_plan.push_back(target_point);
		RCLCPP_INFO_STREAM(get_logger(), "Plan length " << current_plan.size());
		cout << "Orig Plan text " << parsed_log[17] << endl;
		vector<string> vstrings2 = parseText(parsed_log[17], ';');
		originalPlanCost = atof(parseText(vstrings2[0], ' ')[0].c_str()) / 100.0;
		originalPlanDistance = atof(parseText(vstrings2[0], ' ')[1].c_str()) / 100.0;
		RCLCPP_INFO_STREAM(get_logger(), "Orig Plan cost: " << originalPlanCost << " Orig Plan distance = " << originalPlanDistance);
		current_original_plan.push_back(robot_point);
		for(size_t i = 1; i < vstrings2.size(); i++){
			vector<double> point;
			point.push_back(atof(parseText(vstrings2[i], ' ')[0].c_str()));
			point.push_back(atof(parseText(vstrings2[i], ' ')[1].c_str()));
			current_original_plan.push_back(point);
		}
		current_original_plan.push_back(target_point);
		RCLCPP_INFO_STREAM(get_logger(), "Orig Plan length " << current_original_plan.size());
	}

	double computeDistance(double x1, double y1, double x2, double y2){
		//RCLCPP_INFO_STREAM(get_logger(), "Inside compute distance");
		double distance = sqrt(pow((x1 - x2),2) + pow((y1 - y2),2));
		return distance;
	}

	void computePlanDensities(){
		planCrowdDensity = gridCost(current_crowd_density, current_plan);
		originalPlanCrowdDensity =
			gridCost(current_crowd_density, current_original_plan);
	}

	void computePlanRisks(){
		planCrowdRisk = gridCost(current_crowd_risk, current_plan);
		originalPlanCrowdRisk =
			gridCost(current_crowd_risk, current_original_plan);
	}

	double gridCost(
		const nav_msgs::msg::OccupancyGrid& grid,
		const vector<vector<double>>& plan) const
	{
		const double resolution = grid.info.resolution;
		const size_t width = grid.info.width;
		const size_t height = grid.info.height;
		if(
			resolution <= 0.0 || width == 0 || height == 0 ||
			grid.data.size() != width * height)
		{
			return 0.0;
		}
		double total = 0.0;
		for(const vector<double>& point : plan){
			if(point.size() < 2){
				continue;
			}
			const int column = static_cast<int>(floor(
				(point[0] - grid.info.origin.position.x) / resolution));
			const int row = static_cast<int>(floor(
				(point[1] - grid.info.origin.position.y) / resolution));
			if(
				column < 0 || row < 0 ||
				static_cast<size_t>(column) >= width ||
				static_cast<size_t>(row) >= height)
			{
				continue;
			}
			const int value = grid.data[
				static_cast<size_t>(row) * width +
				static_cast<size_t>(column)];
			if(value >= 0){
				total += value;
			}
		}
		return total;
	}

	bool comparePlans(){
		sameplan = 1;
		//RCLCPP_INFO_STREAM(get_logger(), "Comparing plans...");
		if (planDistance != originalPlanDistance){
			sameplan = 0;
			RCLCPP_INFO_STREAM(get_logger(), "Plan distances are different");
			return sameplan;
		}
		else if (planCrowdDensity != originalPlanCrowdDensity or planCrowdRisk != originalPlanCrowdRisk or planCost != originalPlanCost) {
			sameplan = 0;
			RCLCPP_INFO_STREAM(get_logger(), "Plan densities or risks or costs are different");
			return sameplan;
		}
		/*else if (current_plan.size() != current_original_plan.size()) {
			sameplan = 0;
			RCLCPP_INFO_STREAM(get_logger(), "Plan number of poses are different");
			return sameplan;
		}
		else {
			for(size_t i = 0; i < current_plan.size()-1; i++){
				if (current_plan[i][0] != current_original_plan[i][0] or current_plan[i][1] != current_original_plan[i][1]) {
					sameplan = 0;
					RCLCPP_INFO_STREAM(get_logger(), "One of the plan poses is different");
					return sameplan;
				}
			}
		}*/
		RCLCPP_INFO_STREAM(get_logger(), "Plans are the same");
		return sameplan;
	}

	// string densityDifftoPhrase(){
	// 	//RCLCPP_INFO_STREAM(get_logger(), "Inside density diff to phrase");
	// 	string phrase;
	// 	for (size_t i = densityThreshold.size(); i-- > 0;) {
	// 		if ((planCrowdDensity - originalPlanCrowdDensity) <= densityThreshold[i]) {
	// 			phrase = densityPhrase[i];
	// 			densityNum = i;
	// 		}
	// 	}
	// 	RCLCPP_INFO_STREAM(get_logger(), (planCrowdDensity - originalPlanCrowdDensity) << " " << phrase << " " << densityNum);
	// 	return phrase;
	// }

	// string riskDifftoPhrase(){
	// 	//RCLCPP_INFO_STREAM(get_logger(), "Inside risk diff to phrase");
	// 	string phrase;
	// 	for (size_t i = densityThreshold.size(); i-- > 0;) {
	// 		if ((planCrowdRisk - originalPlanCrowdRisk) <= densityThreshold[i]) {
	// 			phrase = densityPhrase[i];
	// 			riskNum = i;
	// 		}
	// 	}
	// 	RCLCPP_INFO_STREAM(get_logger(), (planCrowdRisk - originalPlanCrowdRisk) << " " << phrase << " " << riskNum);
	// 	return phrase;
	// }

	string costDifftoPhrase(string plannerName){
		RCLCPP_INFO_STREAM(get_logger(), "Inside cost diff to phrase");
		auto threshold = thresholds.find(plannerName);
		if(threshold == thresholds.end()){
			RCLCPP_WARN(
				get_logger(),
				"No explanation thresholds registered for planner '%s'; "
				"using distance language",
				plannerName.c_str());
			threshold = thresholds.find("distance");
		}
		if(threshold == thresholds.end()){
			return "somewhat";
		}
		const vector<pair<double, string>>& plannerThreshold =
			threshold->second;
		string phrase;
		for (size_t i = plannerThreshold.size(); i-- > 0;) {
			if ((planCost - originalPlanCost) <= plannerThreshold[i].first) {
				phrase = plannerThreshold[i].second;
				costNum = i;
			}
		}
		RCLCPP_INFO_STREAM(get_logger(), (planCost - originalPlanCost) << " " << phrase << " " << costNum);
		return phrase;
	}

	string distanceDiffToPhrase(string plannerName){
		RCLCPP_INFO_STREAM(get_logger(), "Inside distance diff to phrase");
		auto threshold = thresholds.find(plannerName);
		if(threshold == thresholds.end()){
			threshold = thresholds.find("distance");
		}
		if(threshold == thresholds.end()){
			return "somewhat";
		}
		const vector<pair<double, string>>& plannerThreshold =
			threshold->second;
		string phrase;
		for (size_t i = plannerThreshold.size(); i-- > 0;) {
			if ((planDistance - originalPlanDistance) <= plannerThreshold[i].first) {
				phrase = plannerThreshold[i].second;
				distanceNum = i;
			}
		}
		RCLCPP_INFO_STREAM(get_logger(), (planDistance - originalPlanDistance) << " " << phrase << " " << distanceNum);
		return phrase;
	}

	string objectivePhrase(string plannerName, int type){
		RCLCPP_INFO_STREAM(get_logger(), "Inside objectivePhrase");
		auto objective = objectives.find(plannerName);
		if(objective == objectives.end()){
			RCLCPP_WARN(
				get_logger(),
				"No explanation language registered for planner '%s'; "
				"using distance language",
				plannerName.c_str());
			objective = objectives.find("distance");
		}
		if(
			objective == objectives.end() ||
			type < 0 ||
			static_cast<size_t>(type) >= objective->second.size())
		{
			return "effective";
		}
		const string& phrase = objective->second[static_cast<size_t>(type)];
		RCLCPP_INFO_STREAM(get_logger(), type << " " << phrase);
		return phrase;
	}

	// string computeConf(){
	// 	RCLCPP_INFO_STREAM(get_logger(), "Inside compute conf");
	// 	string phrase, tempphrase;
	// 	if (densityNum == 5) tempphrase = densityDifftoPhrase();
	// 	if (distanceNum == 5) tempphrase = distanceDiffToPhrase();
	// 	RCLCPP_INFO_STREAM(get_logger(), (planCrowdDensity - originalPlanCrowdDensity) << " " << densityNum << " " << (planDistance - originalPlanDistance) << " " << distanceNum);
	// 	if ((densityNum == 0 and distanceNum == 2) or (densityNum == 1 and distanceNum == 1) or (densityNum == 2 and distanceNum == 0)){
	// 		phrase = "only somewhat sure because even though my plan is " + densityDifftoPhrase() + " less crowded, it is also " + distanceDiffToPhrase() + " longer than your plan";
	// 	}
	// 	else if((densityNum == 1 and distanceNum == 2) or (densityNum == 2 and distanceNum == 2) or (densityNum == 2 and distanceNum == 1)){
	// 		phrase = "not sure because my plan is " + distanceDiffToPhrase() + " longer than your plan and only " + densityDifftoPhrase() + " less crowded";
	// 	}
	// 	else if((densityNum == 0 and distanceNum == 1) or (densityNum == 0 and distanceNum == 0) or (densityNum == 1 and distanceNum == 0)){
	// 		phrase = "really sure because my plan is " + densityDifftoPhrase() + " less crowded and only " + distanceDiffToPhrase() + " longer than your plan";
	// 	}
	// 	return phrase;
	// }

	// string computeRiskConf(){
	// 	RCLCPP_INFO_STREAM(get_logger(), "Inside compute risk conf");
	// 	string phrase, tempphrase;
	// 	if (riskNum == 5) tempphrase = riskDifftoPhrase();
	// 	if (distanceNum == 5) tempphrase = distanceDiffToPhrase();
	// 	RCLCPP_INFO_STREAM(get_logger(), (planCrowdRisk - originalPlanCrowdRisk) << " " << riskNum << " " << (planDistance - originalPlanDistance) << " " << distanceNum);
	// 	if ((riskNum == 0 and distanceNum == 2) or (riskNum == 1 and distanceNum == 1) or (riskNum == 2 and distanceNum == 0)){
	// 		phrase = "only somewhat sure because even though my plan is " + riskDifftoPhrase() + " less risky, it is also " + distanceDiffToPhrase() + " longer than your plan";
	// 	}
	// 	else if((riskNum == 1 and distanceNum == 2) or (riskNum == 2 and distanceNum == 2) or (riskNum == 2 and distanceNum == 1)){
	// 		phrase = "not sure because my plan is " + distanceDiffToPhrase() + " longer than your plan and only " + riskDifftoPhrase() + " less risky";
	// 	}
	// 	else if((riskNum == 0 and distanceNum == 1) or (riskNum == 0 and distanceNum == 0) or (riskNum == 1 and distanceNum == 0)){
	// 		phrase = "really sure because my plan is " + riskDifftoPhrase() + " less risky and only " + distanceDiffToPhrase() + " longer than your plan";
	// 	}
	// 	return phrase;
	// }

	string computeCostConf(){
		//RCLCPP_INFO_STREAM(get_logger(), "Inside compute cost conf");
		string phrase, tempphrase;
		if (costNum == 5) tempphrase = costDifftoPhrase(selected_planner);
		if (distanceNum == 5) tempphrase = distanceDiffToPhrase(alt_planner);
		//RCLCPP_INFO_STREAM(get_logger(), (planCost - originalPlanCost) << " " << costNum << " " << (planDistance - originalPlanDistance) << " " << distanceNum);
		if ((costNum == 0 and distanceNum == 2) or (costNum == 1 and distanceNum == 1) or (costNum == 2 and distanceNum == 0)){
			phrase = "only somewhat sure because even though my plan is " + costDifftoPhrase(selected_planner) + " " + objectivePhrase(selected_planner, 1) + ", it is also " + distanceDiffToPhrase(alt_planner) + " " + objectivePhrase(alt_planner, 2) + " than your plan";
		}
		else if((costNum == 1 and distanceNum == 2) or (costNum == 2 and distanceNum == 2) or (costNum == 2 and distanceNum == 1)){
			phrase = "not sure because my plan is " + distanceDiffToPhrase(alt_planner) + " " + objectivePhrase(alt_planner, 1) + " than your plan and only " + costDifftoPhrase(selected_planner) + " " + objectivePhrase(selected_planner, 2) + " than your plan";
		}
		else if((costNum == 0 and distanceNum == 1) or (costNum == 0 and distanceNum == 0) or (costNum == 1 and distanceNum == 0)){
			phrase = "really sure because my plan is " + costDifftoPhrase(selected_planner) + " " + objectivePhrase(selected_planner, 1) + " and only " + distanceDiffToPhrase(alt_planner) + " " + objectivePhrase(alt_planner, 2) + " than your plan";
		}
		return phrase;
	}

	vector<string> parseText(string text, char delim){
		//RCLCPP_INFO_STREAM(get_logger(), "Inside parse text");
		vector<string> vstrings;
		stringstream ss;
		ss.str(text);
		string item;
		// char delim = '\t';
		while (getline(ss, item, delim)) {
			vstrings.push_back(item);
		}
		return vstrings;
	}
	
	void clearStats() {
		//RCLCPP_INFO_STREAM(get_logger(), "Inside clear stats");
		planDistance=0, originalPlanDistance=0, planCrowdDensity=0, originalPlanCrowdDensity=0, planCrowdRisk=0, originalPlanCrowdRisk=0, planCost=0, originalPlanCost=0;
		targetX=0, targetY=0, robotY=0, robotX=0;
		sameplan=1;
		computationTimeSec=0.0;
		densityNum=5, distanceNum=5, riskNum=5, costNum=5;
		string selected_planner;
		alternative_planners.clear();
		alt_planner = " ";
		current_plan.clear();
		current_original_plan.clear();
	}
	
	void logExplanationData() {
		//RCLCPP_INFO_STREAM(get_logger(), "Inside log explanation data");
		std_msgs::msg::String logData;
		vector<string> vstrings = parseText(current_log, '\t');

		stringstream output;
		output << atof(vstrings[0].c_str()) << "\t" << atof(vstrings[1].c_str()) << "\t" << atof(vstrings[2].c_str()) << "\t" << computationTimeSec << "\t" << sameplan << "\t" << alt_planner << "\t" << planDistance << "\t" << originalPlanDistance << "\t" << (planDistance - originalPlanDistance) << "\t" << selected_planner << "\t" << planCost << "\t" << originalPlanCost << "\t" << (planCost - originalPlanCost) << "\t" << vstrings[16] << "\t" << vstrings[17];
		
		logData.data = output.str();
		plan_explanations_log_pub_->publish(logData);
	}
};

// Main file : Load configuration file

int main(int argc, char **argv) {

	rclcpp::init(argc, argv);
	try {
		auto explain = make_shared<Explanation>();
		const string default_config =
			ament_index_cpp::get_package_share_directory("why_plan") +
			"/config/text.conf";
		explain->declare_parameter<string>("text_config", default_config);
		explain->initialize(
			explain->get_parameter("text_config").as_string());
		RCLCPP_INFO(explain->get_logger(), "Plan explanation initialized");
		explain->run();
		if(rclcpp::ok()){
			rclcpp::shutdown();
		}
		return 0;
	} catch (const exception & error) {
		if(!rclcpp::ok()){
			return 0;
		}
		auto logger = rclcpp::get_logger("why_plan");
		RCLCPP_FATAL(
			logger, "Failed to start plan explanation node: %s", error.what());
		rclcpp::shutdown();
		return 1;
	}
}
