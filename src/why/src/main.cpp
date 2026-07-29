/* \mainpage SemaFORR Explanation
 * \brief Explains low-level actions of a robot.
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
#include <stdexcept>
#include <sys/time.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <functional>
#include <memory>

using namespace std;

class Explanation : public rclcpp::Node
{
private:
	//! We will be publishing to the "explanations" topic
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr explanations_pub_;
	//! We will be publishing to the "explanations_log" topic
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr explanations_log_pub_;
	//! We will be listening to \decision_log topic
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_decisionLog_;
	// Current log
	string current_log;
	// Message received
	bool init_message_received;
	// Actions with their associated phrases
	map <string, string> actionText;
	map <string, string> actioningText;
	// t-score intervals with their associated phrases
	vector <double> tScoreThreshold;
	vector <string> tScorePhrase;
	// Advisors with their associated rationales
	map <string, string> advSupportRationales;
	map <string, string> advOpposeRationales;
	// Confidence metrics with their associated phrases
	vector <double> confidenceLevelThreshold;
	vector <string> confidenceLevelPhrase;
	vector <double> giniThreshold;
	vector <string> giniPhrase;
	vector <double> overallSupportThreshold;
	vector <string> overallSupportPhrase;
	// Tier 3 alternate action phrase
	vector <double> diffOverallSupportThreshold;
	vector <string> diffOverallSupportPhrase;
	// Stats on tier 3
	set<string> advisors;
	map <string, double> advisorTotal;
	map <string, double> advisorCount;
	map <string, double> advisorMean;
	map <string, double> advisorStandardDeviation;
	map <string, double> advisorTScore;
	set<string> actions;
	map <string, double> actionTotal;
	map <string, double> actionCount;
	map <string, double> actionMean;
	map <string, double> actionStandardDeviation;
	double totalCommentCount=0, totalCommentMean=0, totalCommentStdev=0;
	double gini, overallSupport, confidenceLevel;
	vector <double> diffTScores, diffOverallSupports;
	double computationTimeSec=0.0;
	double decisionTier=0;

public:
	//! ROS node initialization
	Explanation() : rclcpp::Node("why")
	{
		declare_parameter<string>("decision_log_topic", "decision_log");
		declare_parameter<string>("explanations_topic", "explanations");
		declare_parameter<string>(
			"explanations_log_topic", "explanations_log");
		explanations_pub_ = create_publisher<std_msgs::msg::String>(
			get_parameter("explanations_topic").as_string(), 10);
		explanations_log_pub_ = create_publisher<std_msgs::msg::String>(
			get_parameter("explanations_log_topic").as_string(), 10);
		sub_decisionLog_ = create_subscription<std_msgs::msg::String>(
			get_parameter("decision_log_topic").as_string(),
			100,
			bind(&Explanation::updateLog, this, placeholders::_1));
		init_message_received = false;
	}

	void validateConfigGroup(
		const vector<string>& fields,
		size_t group_size,
		const string& key) const
	{
		if(fields.size() <= 1 || (fields.size() - 1) % group_size != 0){
			throw runtime_error(
				"Malformed '" + key + "' entry in explanation text config");
		}
	}

	void updateLog(const std_msgs::msg::String::ConstSharedPtr log){
		const vector<string> fields = parseText(log->data, '\t');
		if(fields.size() < 17){
			RCLCPP_WARN(
				get_logger(), "Ignoring malformed decision diagnostics");
			return;
		}
		init_message_received = true;
		current_log = log->data;
		//RCLCPP_INFO_STREAM(get_logger(), "Recieved log data: " << current_log << endl);
	}

	void initialize(string text_config){
		string fileLine;
		ifstream file(text_config.c_str());
		RCLCPP_DEBUG_STREAM(get_logger(), "Reading text_config_file:" << text_config);
		if(!file.is_open()){
			throw runtime_error(
				"Unable to locate or read explanation text config: " +
				text_config);
		}

		while(getline(file, fileLine)){
			//cout << "Inside while in tasks" << endl;
			if(fileLine.empty() || fileLine[0] == '#')  // skip comment lines
				continue;
			else if (fileLine.find("actiontext") != string::npos){
				vector<string> vstrings = parseText(fileLine, '\t');
				validateConfigGroup(vstrings, 2, "actiontext");
				for(size_t i=1; i < vstrings.size(); i+=2){
					actionText.insert( pair<string,string>(vstrings[i],vstrings[i+1]));
					//RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[i] << " " << vstrings[i+1] << endl);
				}
				//RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[0]);
			}
			else if (fileLine.find("tscorephrase") != string::npos){
				vector<string> vstrings = parseText(fileLine, '\t');
				validateConfigGroup(vstrings, 2, "tscorephrase");
				for(size_t i=1; i < vstrings.size(); i+=2){
					tScoreThreshold.push_back(atof(vstrings[i].c_str()));
					tScorePhrase.push_back(vstrings[i+1]);
					//RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[i+1] << " " << vstrings[i] << endl);
				}
				//RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[0]);
			}
			else if (fileLine.find("advsupportrationales") != string::npos){
				vector<string> vstrings = parseText(fileLine, '\t');
				validateConfigGroup(vstrings, 2, "advsupportrationales");
				for(size_t i=1; i < vstrings.size(); i+=2){
					advSupportRationales.insert( pair<string,string>(vstrings[i],vstrings[i+1]));
					//RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[i] << " " << vstrings[i+1] << endl);
				}
			}
			else if (fileLine.find("advopposerationales") != string::npos){
				vector<string> vstrings = parseText(fileLine, '\t');
				validateConfigGroup(vstrings, 2, "advopposerationales");
				for(size_t i=1; i < vstrings.size(); i+=2){
					advOpposeRationales.insert( pair<string,string>(vstrings[i],vstrings[i+1]));
					//RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[i] << " " << vstrings[i+1] << endl);
				}
			}
			else if (fileLine.find("confidencelevel") != string::npos){
				vector<string> vstrings = parseText(fileLine, '\t');
				validateConfigGroup(vstrings, 2, "confidencelevel");
				for(size_t i=1; i < vstrings.size(); i+=2){
					confidenceLevelThreshold.push_back(atof(vstrings[i].c_str()));
					confidenceLevelPhrase.push_back(vstrings[i+1]);
					//RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[i+1] << " " << vstrings[i] << endl);
				}
				//RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[0]);
			}
			else if (fileLine.find("gini") != string::npos){
				vector<string> vstrings = parseText(fileLine, '\t');
				validateConfigGroup(vstrings, 2, "gini");
				for(size_t i=1; i < vstrings.size(); i+=2){
					giniThreshold.push_back(atof(vstrings[i].c_str()));
					giniPhrase.push_back(vstrings[i+1]);
					//RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[i+1] << " " << vstrings[i] << endl);
				}
				//RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[0]);
			}
			else if (fileLine.find("overallsupport") != string::npos){
				vector<string> vstrings = parseText(fileLine, '\t');
				validateConfigGroup(vstrings, 2, "overallsupport");
				for(size_t i=1; i < vstrings.size(); i+=2){
					overallSupportThreshold.push_back(atof(vstrings[i].c_str()));
					overallSupportPhrase.push_back(vstrings[i+1]);
					//RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[i+1] << " " << vstrings[i] << endl);
				}
				//RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[0]);
			}
			else if (fileLine.find("diffOS") != string::npos){
				vector<string> vstrings = parseText(fileLine, '\t');
				validateConfigGroup(vstrings, 2, "diffOS");
				for(size_t i=1; i < vstrings.size(); i+=2){
					diffOverallSupportThreshold.push_back(atof(vstrings[i].c_str()));
					diffOverallSupportPhrase.push_back(vstrings[i+1]);
					//RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[i+1] << " " << vstrings[i] << endl);
				}
				//RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[0]);
			}
			else if (fileLine.find("actioningtext") != string::npos){
				vector<string> vstrings = parseText(fileLine, '\t');
				validateConfigGroup(vstrings, 2, "actioningtext");
				for(size_t i=1; i < vstrings.size(); i+=2){
					actioningText.insert( pair<string,string>(vstrings[i],vstrings[i+1]));
					//RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[i] << " " << vstrings[i+1] << endl);
				}
				//RCLCPP_DEBUG_STREAM(get_logger(), "File text:" << vstrings[0]);
			}
		}
		if(
			actionText.empty() || actioningText.empty() ||
			tScoreThreshold.empty() || confidenceLevelThreshold.empty() ||
			giniThreshold.empty() || overallSupportThreshold.empty())
		{
			throw runtime_error(
				"Explanation text config is missing required entries");
		}


		rclcpp::spin_some(shared_from_this());
	}
	
	void run(){
		std_msgs::msg::String explanationString;
		string vetoedActions, chosenAction, advisorComments;
		vector<string> currentPlan;
		rclcpp::Rate rate(30.0);
		timeval cv;
		double start_timecv, end_timecv;
		while(rclcpp::ok()) {
			while(rclcpp::ok() && init_message_received == false){
				// ROS_DEBUG("Waiting for first message");
				//wait for some time
				rate.sleep();
				// Sense input 
				rclcpp::spin_some(shared_from_this());
			}
			if(!rclcpp::ok()){
				break;
			}
			gettimeofday(&cv,NULL);
			start_timecv = cv.tv_sec + (cv.tv_usec/1000000.0);
			//target = "(" + parseText(current_log, '\t')[4] + ", " + parseText(current_log, '\t')[5] + ")";
			decisionTier = atof(parseText(current_log, '\t')[10].c_str());
			vetoedActions = parseText(current_log, '\t')[11];
			chosenAction = parseText(current_log, '\t')[12]+parseText(current_log, '\t')[13];
			advisorComments = parseText(current_log, '\t')[15];
			currentPlan = parseText(parseText(current_log, '\t')[16], ';');
			// RCLCPP_INFO_STREAM(get_logger(), decisionTier << " " << vetoedActions << " " << chosenAction << " " << advisorComments << endl << endl);
			vector< vector <string> > vetoes;
			stringstream ss;
			ss.str(vetoedActions);
			string item;
			char delim = ';';
			while (getline(ss, item, delim)) {
				vector<string> vstrings;
				stringstream st;
				st.str(item);
				string sitem;
				char sdelim = ' ';
				while (getline(st, sitem, sdelim)) {
					vstrings.push_back(sitem);
				}
				vetoes.push_back(vstrings);
			}
			int numMovesVetoed = 0;
			int numRotationsVetoed = 0;
			for(size_t i = 0; i < vetoes.size(); i++){
				if(vetoes[i][0] == "0"){
					numMovesVetoed ++;
				}
				else if(vetoes[i][0] == "1" or vetoes[i][0] == "2"){
					numRotationsVetoed ++;
				}
			}
			// enforcer2, thru3, behind4, out5, lle6
			cout << "numMovesVetoed " << numMovesVetoed << " numRotationsVetoed " << numRotationsVetoed << endl;
			if (decisionTier == 1.1){
				explanationString.data = "I could see our target and " + actioningText[chosenAction] + " would get us closer to it.\n" + "Highly confident, since our target is in sensor range and this would get us closer to it.\n" + alternateActions(chosenAction, decisionTier, vetoes);
			}
			else if (decisionTier == 1.2){
				explanationString.data = "I could see our waypoint and " + actioningText[chosenAction] + " would get us closer to it.\n" + "Highly confident, since our waypoint is in sensor range and this would get us closer to it.\n" + alternateActions(chosenAction, decisionTier, vetoes);
			}
			else if (decisionTier == 1.21){
				explanationString.data = "I saw that " + actioningText[chosenAction] + " would let us take a shortcut to get further along in our plan.\n" + "Highly confident, since our next waypoint is in sensor range and this would get us closer to it.\n" + alternateActions(chosenAction, decisionTier, vetoes);
			}
			else if (decisionTier == 1.3){
				explanationString.data = "I can't get where I want to go and " + actioningText[chosenAction] + " helps me reposition to get there.\n" + "Somewhat confident, because I am not sure this would get me through.\n" + alternateActions(chosenAction, decisionTier, vetoes);
			}
			else if (decisionTier == 1.4){
				explanationString.data = "I think where I want to go is behind me and " + actioningText[chosenAction] + " will help me see it.\n" + "Somewhat confident, because I am not sure this shows me the way.\n" + alternateActions(chosenAction, decisionTier, vetoes);
			}
			else if (decisionTier == 1.5){
				explanationString.data = "I am " + actioningText[chosenAction] + " because I'm stuck and want to get out of here.\n" + "Somewhat confident, because I am not sure if this will get me out.\n" + alternateActions(chosenAction, decisionTier, vetoes);
			}
			else if (decisionTier == 1.6){
				explanationString.data = "I want to get closer to our target and " + actioningText[chosenAction] + " would let us explore in that direction.\n" + "Somewhat confident, because I am not sure if this is the right way to our target.\n" + alternateActions(chosenAction, decisionTier, vetoes);
			}
			else if (decisionTier == 1.7){
				cout << "inside decisionTier 1.7" << endl;
				explanationString.data = "I want to learn about our world and " + actioningText[chosenAction] + " would let me explore.\n" + "Somewhat confident, because I am not sure if this area will help me get around later.\n" + alternateActions(chosenAction, decisionTier, vetoes);
			}
			else if (decisionTier == 1.8){
				explanationString.data = "I want to find the boundaries of our world and " + actioningText[chosenAction] + " would let find them.\n" + "Somewhat confident, because I am not sure if I will need to know about these boundaries later.\n" + alternateActions(chosenAction, decisionTier, vetoes);
			}
			else if (numMovesVetoed == 6 and numRotationsVetoed == 12 and chosenAction == "30") {
				//ROS_DEBUG(vetoedActions << endl);
				decisionTier = 1;
				explanationString.data = "I decided to " + actionText[chosenAction] + " because I want to think more about what to do.\n" + "Not confident, since I don't know what else to do right now.\n" + vetoedAlternateActions(vetoes, chosenAction);
			}
			else {
				parseTier3Comments(advisorComments);
				advisorTScore = computeTier3TScores(advisorComments, chosenAction);
				computeConfidence(chosenAction);
				explanationString.data = tier3Explanation(chosenAction) + "\n" + confidenceExplanation() + "\n" + vetoedAlternateActions(vetoes, chosenAction) + "\n" + tier3AlternateActions(advisorComments, chosenAction);
				if(currentPlan.size() > 1){
					string from = "target";
					string to = "waypoint";
					string::size_type start_pos = 0;
					while((start_pos = explanationString.data.find(from, start_pos)) != string::npos) {
						explanationString.data.replace(start_pos, from.length(), to);
						start_pos += to.length();
					}
				}
			}
			gettimeofday(&cv,NULL);
			end_timecv = cv.tv_sec + (cv.tv_usec/1000000.0);
			computationTimeSec = (end_timecv-start_timecv);
			RCLCPP_INFO_STREAM(get_logger(), "After explanation: " << explanationString.data);
			logExplanationData();
			//send the explanation
			explanations_pub_->publish(explanationString);
			init_message_received = false;
			clearStats();
			//wait for some time
			rate.sleep();
			// Sense input 
			rclcpp::spin_some(shared_from_this());
		}
	}

	void parseTier3Comments(string advisorComments) {
		stringstream ss1;
		ss1.str(advisorComments);
		string item, val;
		char delim = ';';
		while (getline(ss1, item, delim)) {
			stringstream subss;
			subss.str(item);
			char delim2 = ' ';
			vector<string> vstrings;
			while (getline(subss, val, delim2)) {
				vstrings.push_back(val);
			}
			string action = (vstrings[1]+vstrings[2]);
			if (advisorTotal.find(vstrings[0]) == advisorTotal.end()) {
				advisorTotal[vstrings[0]] = atof(vstrings[3].c_str());
			} else {
				advisorTotal[vstrings[0]] = advisorTotal[vstrings[0]] + atof(vstrings[3].c_str());
			}
			if (advisorCount.find(vstrings[0]) == advisorCount.end()) {
				advisorCount[vstrings[0]] = 1;
			} else {
				advisorCount[vstrings[0]] = advisorCount[vstrings[0]] + 1;
			}
			advisors.insert(vstrings[0]);
			if (actionTotal.find(action) == actionTotal.end()) {
				actionTotal[action] = atof(vstrings[3].c_str());
			} else {
				actionTotal[action] = actionTotal[action] + atof(vstrings[3].c_str());
			}
			if (actionCount.find(action) == actionCount.end()) {
				actionCount[action] = 1;
			} else {
				actionCount[action] = actionCount[action] + 1;
			}
			actions.insert(action);
			//RCLCPP_INFO_STREAM(get_logger(), vstrings[0] << " " << action << " " << vstrings[3] << ";");
			vstrings.clear();
		}
		set<string>::iterator adv, act;
		for (adv = advisors.begin(); adv != advisors.end(); adv++) {
			advisorMean[*adv] = (advisorTotal[*adv] / advisorCount[*adv]);
		}
		for (act = actions.begin(); act != actions.end(); act++) {
			actionMean[*act] = (actionTotal[*act] / actionCount[*act]);
		}

		stringstream ss2;
		ss2.str(advisorComments);
		while (getline(ss2, item, delim)) {
			stringstream subss;
			subss.str(item);
			char delim2 = ' ';
			vector<string> vstrings;
			while (getline(subss, val, delim2)) {
				vstrings.push_back(val);
			}
			string action = (vstrings[1]+vstrings[2]);
			if (advisorStandardDeviation.find(vstrings[0]) == advisorStandardDeviation.end()) {
				advisorStandardDeviation[vstrings[0]] = pow((atof(vstrings[3].c_str()) - advisorMean[vstrings[0]]), 2);
			} else {
				advisorStandardDeviation[vstrings[0]] = advisorStandardDeviation[vstrings[0]] + pow((atof(vstrings[3].c_str()) - advisorMean[vstrings[0]]), 2);
			}
			
			if (actionStandardDeviation.find(action) == actionStandardDeviation.end()) {
				actionStandardDeviation[action] = pow((atof(vstrings[3].c_str()) - actionMean[action]), 2);
			} else {
				actionStandardDeviation[action] = actionStandardDeviation[action] + pow((atof(vstrings[3].c_str()) - actionMean[action]), 2);
			}
			vstrings.clear();
		}
		for (adv = advisors.begin(); adv != advisors.end(); adv++) {
			advisorStandardDeviation[*adv] = sqrt(advisorStandardDeviation[*adv] / advisorCount[*adv]);
		}
		for (act = actions.begin(); act != actions.end(); act++) {
			actionStandardDeviation[*act] = sqrt(actionStandardDeviation[*act] / actionCount[*act]);
		}
	}

	map <string, double> computeTier3TScores(string advisorComments, string chosenAction) {
		map <string, double> tScores;
		stringstream ss1;
		ss1.str(advisorComments);
		string item, val;
		char delim = ';';
		while (getline(ss1, item, delim)) {
			stringstream subss;
			subss.str(item);
			char delim2 = ' ';
			vector<string> vstrings;
			while (getline(subss, val, delim2)) {
				vstrings.push_back(val);
			}
			string action = (vstrings[1]+vstrings[2]);
			if (action == chosenAction) {
				if (advisorStandardDeviation[vstrings[0]] != 0) {
					tScores[vstrings[0]] = ((atof(vstrings[3].c_str()) - advisorMean[vstrings[0]]) / advisorStandardDeviation[vstrings[0]]);
					//RCLCPP_INFO_STREAM(get_logger(), vstrings[0] << " " << action << ": " << vstrings[3] << ", mean: " << advisorMean[vstrings[0]] << ", stdev: " << advisorStandardDeviation[vstrings[0]] << ", t score: " << (atof(vstrings[3].c_str()) - advisorMean[vstrings[0]]) / advisorStandardDeviation[vstrings[0]]);
				} else {
					tScores[vstrings[0]] = 0;
				}
			}
			vstrings.clear();
		}
		return tScores;
	}

	string tier3TScoretoPhrase(double tscore) {
		string phrase;
		for (size_t i = tScoreThreshold.size(); i-- > 0;) {
			if (tscore <= tScoreThreshold[i]) {
				phrase = tScorePhrase[i];
			}
		}
		//RCLCPP_INFO_STREAM(get_logger(), tscore << " " << phrase);
		return phrase;
	}

	string tier3Explanation(string chosenAction) {
		string explanation, supportConcat, opposeConcat;
		vector<string> supportPhrases, slightSupportPhrases;
		vector<string> opposePhrases, slightOpposePhrases;
		
		map <string, double>::iterator itr;
		//RCLCPP_INFO_STREAM(get_logger(), advisorTScore.size());
		for (itr = advisorTScore.begin(); itr != advisorTScore.end(); itr++) {
			if (itr->second > (1.5)) {
				if(supportPhrases.size() == 0){
					supportPhrases.push_back("I " + tier3TScoretoPhrase(itr->second) + " to " + advSupportRationales[itr->first]);
				}
				else{
					supportPhrases.push_back(advSupportRationales[itr->first]);
				}
				//RCLCPP_INFO_STREAM(get_logger(), itr->first << ": " << itr->second);
				//RCLCPP_INFO_STREAM(get_logger(), "I " + tier3TScoretoPhrase(itr->second) + " to " + advSupportRationales[itr->first]);
			}
			else if (itr->second > (0.75)) {
				if(slightSupportPhrases.size() == 0){
					slightSupportPhrases.push_back("I " + tier3TScoretoPhrase(itr->second) + " to " + advSupportRationales[itr->first]);
				}
				else{
					slightSupportPhrases.push_back(advSupportRationales[itr->first]);
				}
				//RCLCPP_INFO_STREAM(get_logger(), itr->first << ": " << itr->second);
				//RCLCPP_INFO_STREAM(get_logger(), "I " + tier3TScoretoPhrase(itr->second) + " to " + advSupportRationales[itr->first]);
			}
			else if (itr->second > (-1.5) and itr->second <= (-0.75)) {
				if(slightOpposePhrases.size() == 0){
					slightOpposePhrases.push_back("I " + tier3TScoretoPhrase(itr->second) + " to " + advOpposeRationales[itr->first]);
				}
				else{
					slightOpposePhrases.push_back(advOpposeRationales[itr->first]);
				}
				//RCLCPP_INFO_STREAM(get_logger(), itr->first << ": " << itr->second);
				//RCLCPP_INFO_STREAM(get_logger(), "I " + tier3TScoretoPhrase(itr->second) + " to " + advOpposeRationales[itr->first]);
			}
			else if (itr->second <= (-1.5)){
				if(opposePhrases.size() == 0){
					opposePhrases.push_back("I " + tier3TScoretoPhrase(itr->second) + " to " + advOpposeRationales[itr->first]);
				}
				else{
					opposePhrases.push_back(advOpposeRationales[itr->first]);
				}
				//RCLCPP_INFO_STREAM(get_logger(), itr->first << ": " << itr->second);
				//RCLCPP_INFO_STREAM(get_logger(), "I " + tier3TScoretoPhrase(itr->second) + " to " + advOpposeRationales[itr->first]);
			}
		}
		
		if (supportPhrases.size() > 2) {
			for (size_t i = 0; i < supportPhrases.size()-2; i++) {
				supportConcat = supportConcat + supportPhrases[i] + ", ";
			}
			supportConcat = supportConcat + supportPhrases[supportPhrases.size()-2] + " and " + supportPhrases[supportPhrases.size()-1];
			//RCLCPP_INFO_STREAM(get_logger(), "Greater than 2: " << supportConcat);
		}
		else if (supportPhrases.size() == 2) {
			supportConcat = supportPhrases[0] + " and " + supportPhrases[1];
			//RCLCPP_INFO_STREAM(get_logger(), "Equals 2: " << supportConcat);
		}
		else if (supportPhrases.size() == 1) {
			supportConcat = supportPhrases[0];
			//RCLCPP_INFO_STREAM(get_logger(), "Equals 1: " << supportConcat);
		}
		else if (supportPhrases.size() == 0) {
			if (slightSupportPhrases.size() > 2) {
				for (size_t i = 0; i < slightSupportPhrases.size()-2; i++) {
					supportConcat = supportConcat + slightSupportPhrases[i] + ", ";
				}
				supportConcat = supportConcat + slightSupportPhrases[slightSupportPhrases.size()-2] + " and " + slightSupportPhrases[slightSupportPhrases.size()-1];
				//RCLCPP_INFO_STREAM(get_logger(), "Greater than 2 Slightly: " << supportConcat);
			}
			else if (slightSupportPhrases.size() == 2) {
				supportConcat = slightSupportPhrases[0] + " and " + slightSupportPhrases[1];
				//RCLCPP_INFO_STREAM(get_logger(), "Equals 2 Slightly: " << supportConcat);
			}
			else if (slightSupportPhrases.size() == 1) {
				supportConcat = slightSupportPhrases[0];
				//RCLCPP_INFO_STREAM(get_logger(), "Equals 1 Slightly: " << supportConcat);
			}
		}
		
		if (opposePhrases.size() > 2) {
			for (size_t i = 0; i < opposePhrases.size()-2; i++) {
				opposeConcat = opposeConcat + opposePhrases[i] + ", ";
			}
			opposeConcat = opposeConcat + opposePhrases[opposePhrases.size()-2] + " and " + opposePhrases[opposePhrases.size()-1];
			//RCLCPP_INFO_STREAM(get_logger(), "Greater than 2 Oppose: " << opposeConcat);
			// explanation = "Although " + opposeConcat + ", I decided to " + actionText[chosenAction] + " because " + supportConcat + ".";
			//RCLCPP_INFO_STREAM(get_logger(), explanation);
		}
		else if (opposePhrases.size() == 2) {
			opposeConcat = opposePhrases[0] + " and " + opposePhrases[1];
			//RCLCPP_INFO_STREAM(get_logger(), "Equals 2 Oppose: " << opposeConcat);
			// explanation = "Although " + opposeConcat + ", I decided to " + actionText[chosenAction] + " because " + supportConcat + ".";
			//RCLCPP_INFO_STREAM(get_logger(), explanation);
		}
		else if (opposePhrases.size() == 1) {
			opposeConcat = opposePhrases[0];
			//RCLCPP_INFO_STREAM(get_logger(), "Equals 1 Oppose: " << opposeConcat);
			// explanation = "Although " + opposeConcat + ", I decided to " + actionText[chosenAction] + " because " + supportConcat + ".";
			//RCLCPP_INFO_STREAM(get_logger(), explanation);
		}
		else if (opposePhrases.size() == 0) {
			if (slightOpposePhrases.size() > 2) {
				for (size_t i = 0; i < slightOpposePhrases.size()-2; i++) {
					opposeConcat = opposeConcat + slightOpposePhrases[i] + ", ";
				}
				opposeConcat = opposeConcat + slightOpposePhrases[slightOpposePhrases.size()-1] + " and " + slightOpposePhrases[slightOpposePhrases.size()-1];
				//RCLCPP_INFO_STREAM(get_logger(), "Greater than 2 Slightly Oppose: " << opposeConcat);
			}
			else if (slightOpposePhrases.size() == 2) {
				opposeConcat = slightOpposePhrases[0] + " and " + slightOpposePhrases[1];
				//RCLCPP_INFO_STREAM(get_logger(), "Equals 2 Slightly Oppose: " << opposeConcat);
			}
			else if (slightOpposePhrases.size() == 1) {
				opposeConcat = slightOpposePhrases[0];
				//RCLCPP_INFO_STREAM(get_logger(), "Equals 1 Slightly Oppose: " << opposeConcat);
			}
			// explanation = "Although " + opposeConcat + ", I decided to " + actionText[chosenAction] + " because " + supportConcat + ".";
			//RCLCPP_INFO_STREAM(get_logger(), explanation);
		}
		if((supportPhrases.size() > 0 or slightSupportPhrases.size() > 0) and (opposePhrases.size() > 0 or slightOpposePhrases.size() > 0)){
			explanation = "Although " + opposeConcat + ", I decided to " + actionText[chosenAction] + " because " + supportConcat + ".";
		}
		else if (supportPhrases.size() > 0 or slightSupportPhrases.size() > 0){
			explanation = "I decided to " + actionText[chosenAction] + " because " + supportConcat + ".";
			//RCLCPP_INFO_STREAM(get_logger(), explanation);
		}
		else {
			explanation = "I decided to " + actionText[chosenAction] + " because it's just as good as anything else.";
		}
		
		return explanation + "\n";
	}
	
	void computeConfidence(string chosenAction) {
		gini = 2 * (actionTotal[chosenAction]/(10*actionCount[chosenAction])) * (1 - (actionTotal[chosenAction]/(10*actionCount[chosenAction])));
		//RCLCPP_INFO_STREAM(get_logger(), actionTotal[chosenAction] << " " << actionCount[chosenAction] << " " << gini);
		map <string, double>::iterator itr;
		for (itr = actionTotal.begin(); itr != actionTotal.end(); itr++) {
			totalCommentMean += itr->second;
			totalCommentCount++;
		}
		totalCommentMean = totalCommentMean / totalCommentCount;
		for (itr = actionTotal.begin(); itr != actionTotal.end(); itr++) {
			totalCommentStdev += pow((itr->second - totalCommentMean), 2);
		}
		totalCommentStdev = sqrt(totalCommentStdev / totalCommentCount);
		if (totalCommentStdev != 0) {
			overallSupport = (actionTotal[chosenAction] - totalCommentMean)/totalCommentStdev;
		}
		else {
			overallSupport = 0;
		}
		//RCLCPP_INFO_STREAM(get_logger(), totalCommentCount << " " << totalCommentMean << " " << totalCommentStdev << " " << overallSupport);
		confidenceLevel = (0.5 - gini) * overallSupport;
		//RCLCPP_INFO_STREAM(get_logger(), confidenceLevel);
	}
	
	string confidenceExplanation() {
		string explanation, phraseGini, phraseOverallSupport, phraseConfidenceLevel;
		int giniPhraseID = giniThreshold.size(), overallSupportPhraseID = overallSupportThreshold.size(), confidenceLevelPhraseID = confidenceLevelThreshold.size();
		for (size_t i = giniThreshold.size(); i-- > 0;) {
			if (gini <= giniThreshold[i]) {
				phraseGini = giniPhrase[i];
				giniPhraseID--;
			}
		}
		for (size_t i = overallSupportThreshold.size(); i-- > 0;) {
			if (overallSupport <= overallSupportThreshold[i]) {
				phraseOverallSupport = overallSupportPhrase[i];
				overallSupportPhraseID--;
			}
		}
		for (size_t i = confidenceLevelThreshold.size(); i-- > 0;) {
			if (confidenceLevel <= confidenceLevelThreshold[i]) {
				phraseConfidenceLevel = confidenceLevelPhrase[i];
				confidenceLevelPhraseID--;
			}
		}
		giniPhraseID = -(giniPhraseID-2);
		//RCLCPP_INFO_STREAM(get_logger(), giniPhraseID << " " << overallSupportPhraseID << " " << confidenceLevelPhraseID);
		
		if (giniPhraseID == confidenceLevelPhraseID and overallSupportPhraseID == confidenceLevelPhraseID) {
			explanation = "I'm " + phraseConfidenceLevel + " sure in my decision because " + phraseGini + ". I " + phraseOverallSupport + " to do this much more than some other action.";
		}
		else if (giniPhraseID == confidenceLevelPhraseID) {
			explanation = "I'm " + phraseConfidenceLevel + " sure in my decision because " + phraseGini + ".";
		}
		else if (overallSupportPhraseID == confidenceLevelPhraseID) {
			explanation = "I'm " + phraseConfidenceLevel + " sure in my decision because I " + phraseOverallSupport + " to do this much more than some other action.";
		}
		else if (overallSupportPhraseID > confidenceLevelPhraseID and giniPhraseID < confidenceLevelPhraseID) {
			explanation = "I'm " + phraseConfidenceLevel + " sure in my decision because, even though " + phraseGini + ", I " + phraseOverallSupport + " to do this much more than some other action.";
		}
		else if (giniPhraseID > confidenceLevelPhraseID and overallSupportPhraseID < confidenceLevelPhraseID) {
			explanation = "I'm " + phraseConfidenceLevel + " sure in my decision because, even though I " + phraseOverallSupport + " to do this much more than some other action, " + phraseGini + ".";
		}
		else {
			explanation = "I'm " + phraseConfidenceLevel + " sure in my decision because " + phraseGini + ". I " + phraseOverallSupport + " to do this much more than some other action.";
		}		
		//RCLCPP_INFO_STREAM(get_logger(), explanation);
		return explanation + "\n";
	}
	
	string alternateActions(string chosenAction, double decTier, vector< vector <string> > vetoes) {
		cout << "Inside alternateActions" << endl;
		string alternateExplanations;
		map <string, string>::iterator itr;
		if (decTier == 1.1 or decTier == 1.7 or decTier == 1.8){
			for (itr = actionText.begin(); itr != actionText.end(); itr++) {
				if (itr->first != chosenAction and itr->first != "30" and (itr->second).length() >0) {
					if (decTier == 1.1){
						alternateExplanations = alternateExplanations + "I decided not to " + itr->second + " because I sense our goal and another action would get us closer to it.\n";
					}
					else if (decTier == 1.7){
						alternateExplanations = alternateExplanations + "I decided not to " + itr->second + " because another action would let me better explore to learn about the world.\n";
					}
					else if (decTier == 1.8){
						alternateExplanations = alternateExplanations + "I decided not to " + itr->second + " because another action would let me better find the boundaries of this world.\n";
					}
				}
			}
		}
		else{
			for (itr = actionText.begin(); itr != actionText.end(); itr++) {
				bool actionVetoed = false;
				for(size_t i = 0; i < vetoes.size(); i++){
					if(itr->first == vetoes[i][0]+vetoes[i][1]){
						actionVetoed = true;
						if(vetoes[i][2] == "1a"){
							alternateExplanations = alternateExplanations + "I decided not to " + itr->second + " because something was in the way.\n";
						}
						else if(vetoes[i][2] == "1b"){
							alternateExplanations = alternateExplanations + "I decided not to " + itr->second + " because I was just facing that way.\n";
						}
						else if(vetoes[i][2] == "1c"){
							alternateExplanations = alternateExplanations + "I decided not to " + itr->second + " because I've already been there.\n";
						}
						else if(vetoes[i][2] == "1d"){
							alternateExplanations = alternateExplanations + "I decided not to " + itr->second + " because it's not what I usually do when in a spot that looks like this.\n";
						}
					}
				}
				if (itr->first != chosenAction and itr->first != "30" and (itr->second).length() >0 and actionVetoed == false) {
					if (decTier == 1.2){
						alternateExplanations = alternateExplanations + "I decided not to " + itr->second + " because I sense our waypoint and another action would get us closer to it.\n";
					}
					else if (decTier == 1.21){
						alternateExplanations = alternateExplanations + "I decided not to " + itr->second + " because I sense our next waypoint and another action would get us closer to it.\n";
					}
					else if (decTier == 1.3){
						alternateExplanations = alternateExplanations + "I decided not to " + itr->second + " because I need to get through here and another action would reposition me better.\n";
					}
					else if (decTier == 1.4){
						alternateExplanations = alternateExplanations + "I decided not to " + itr->second + " because I want to turn around so I can see where I want to go.\n";
					}
					else if (decTier == 1.5){
						alternateExplanations = alternateExplanations + "I decided not to " + itr->second + " because I think another action will help me get out of here.\n";
					}
					else if (decTier == 1.6){
						alternateExplanations = alternateExplanations + "I decided not to " + itr->second + " because another action would let me better explore for our target.\n";
					}
				}
			}
		}
		return alternateExplanations;
	}
	
	string vetoedAlternateActions(
		vector< vector <string> > vetoes, const string&){
		string alternateExplanations;
		map <string, string>::iterator itr;
		for (itr = actionText.begin(); itr != actionText.end(); itr++) {
			for(size_t i = 0; i < vetoes.size(); i++){
				if(itr->first == vetoes[i][0]+vetoes[i][1]){
					if(vetoes[i][2] == "1a"){
						alternateExplanations = alternateExplanations + "I decided not to " + itr->second + " because something was in the way.\n";
					}
					else if(vetoes[i][2] == "1b"){
						alternateExplanations = alternateExplanations + "I decided not to " + itr->second + " because I was just facing that way.\n";
					}
					else if(vetoes[i][2] == "1c"){
						alternateExplanations = alternateExplanations + "I decided not to " + itr->second + " because I've already been there.\n";
					}
					else if(vetoes[i][2] == "1d"){
						alternateExplanations = alternateExplanations + "I decided not to " + itr->second + " because it's not what I usually do when in a spot that looks like this.\n";
					}
				}
			}
		}
		return alternateExplanations;
	}
	
	string tier3AlternateActions(string advisorComments, string chosenAction) {
		string alternateExplanations;
		map <string, double>::iterator atr;
		for (atr = actionTotal.begin(); atr != actionTotal.end(); atr++) {
			string supportConcat, opposeConcat, phraseDiffOverallSupport;
			vector<string> supportPhrases, opposePhrases;
			if (atr->first != chosenAction) {
				double diffOverallSupport;
				if (totalCommentStdev != 0) {
					diffOverallSupport = overallSupport - ((actionTotal[atr->first] - totalCommentMean)/totalCommentStdev);
				}
				else {
					diffOverallSupport = 0;
				}
				//RCLCPP_INFO_STREAM(get_logger(), diffOverallSupportThreshold.size());
				for (size_t i = diffOverallSupportThreshold.size(); i-- > 0;) {
					//RCLCPP_INFO_STREAM(get_logger(), diffOverallSupport << " " << diffOverallSupportThreshold[i]);
					if (diffOverallSupport <= diffOverallSupportThreshold[i]) {
						phraseDiffOverallSupport = diffOverallSupportPhrase[i];
					}
				}
				//RCLCPP_INFO_STREAM(get_logger(), phraseDiffOverallSupport);
				diffOverallSupports.push_back(diffOverallSupport);
				
				map <string, double> alternateTScores = computeTier3TScores(advisorComments, atr->first);
				map <string, double>::iterator itr;
				for (itr = advisorTScore.begin(); itr != advisorTScore.end(); itr++) {
					if ((itr->second - alternateTScores[itr->first]) > 1) {
						supportPhrases.push_back(advSupportRationales[itr->first]);
					}
					else if ((itr->second - alternateTScores[itr->first]) < -1) {
						opposePhrases.push_back(advSupportRationales[itr->first]);
					}
					diffTScores.push_back((itr->second - alternateTScores[itr->first]));
				}
				
				if (supportPhrases.size() > 2) {
					for (size_t i = 0; i < supportPhrases.size()-1; i++) {
						supportConcat = supportConcat + supportPhrases[i] + ", ";
					}
					supportConcat = supportConcat + "and " + supportPhrases[supportPhrases.size()-1];
				}
				else if (supportPhrases.size() == 2) {
					supportConcat = supportPhrases[0] + " and " + supportPhrases[1];
				}
				else if (supportPhrases.size() == 1) {
					supportConcat = supportPhrases[0];
				}
				
				if (opposePhrases.size() > 2) {
					for (size_t i = 0; i < opposePhrases.size()-1; i++) {
						opposeConcat = opposeConcat + opposePhrases[i] + ", ";
					}
					opposeConcat = opposeConcat + "and " + opposePhrases[opposePhrases.size()-1];
				}
				else if (opposePhrases.size() == 2) {
					opposeConcat = opposePhrases[0] + " and " + opposePhrases[1];
				}
				else if (opposePhrases.size() == 1) {
					opposeConcat = opposePhrases[0];
				}
				
				
				if (supportPhrases.size() > 0 and opposePhrases.size() > 0) {
					alternateExplanations = alternateExplanations + "I thought about " + actioningText[atr->first] + " because it would let us " + opposeConcat + ", but I felt " + phraseDiffOverallSupport + " strongly about " + actioningText[chosenAction] + " since it lets us " + supportConcat + ".\n";
				}
				else if (supportPhrases.size() > 0 and opposePhrases.size() == 0) {
					alternateExplanations = alternateExplanations + "I thought about " + actioningText[atr->first] + ", but I felt " + phraseDiffOverallSupport + " strongly about " + actioningText[chosenAction] + " since it lets us " + supportConcat + ".\n";
				}
				else if (supportPhrases.size() == 0 and opposePhrases.size() > 0) {
					alternateExplanations = alternateExplanations + "I thought about it because " + actioningText[atr->first] + " would let us " + opposeConcat + ", but I felt " + phraseDiffOverallSupport + " strongly about " + actioningText[chosenAction] + ".\n";
				}
				else if (supportPhrases.size() == 0 and opposePhrases.size() == 0) {
					alternateExplanations = alternateExplanations + "I thought about " + actioningText[atr->first] + ", but I felt " + phraseDiffOverallSupport + " strongly about " + actioningText[chosenAction] + ".\n";
				}
				//RCLCPP_INFO_STREAM(get_logger(), alternateExplanations);
			}
		}
		
		return alternateExplanations;
	}

	vector<string> parseText(string text, char delim){
		vector<string> vstrings;
		stringstream ss;
		ss.str(text);
		string item;
		// char delim = '\t';
		while (getline(ss, item, delim)) {
			vstrings.push_back(item);
		}
		//stringstream ss(text);
		//istream_iterator<string> begin(ss);
		//istream_iterator<string> end;
		//vector<string> vstrings(begin, end);
		//RCLCPP_DEBUG_STREAM(get_logger(), "Log text:" << vstrings[0]);
		return vstrings;
	}
	
	void clearStats() {
		advisors.clear();
		advisorTotal.clear();
		advisorCount.clear();
		advisorMean.clear();
		advisorStandardDeviation.clear();
		advisorTScore.clear();
		actions.clear();
		actionTotal.clear();
		actionCount.clear();
		actionMean.clear();
		actionStandardDeviation.clear();
		diffTScores.clear();
		diffOverallSupports.clear();
		
		totalCommentCount=0, totalCommentMean=0, totalCommentStdev=0;
		gini=0, overallSupport=0, confidenceLevel=0;
		computationTimeSec=0.0;
		decisionTier=0;
	}
	
	void logExplanationData() {
		std_msgs::msg::String logData;
		vector<string> vstrings = parseText(current_log, '\t');

		stringstream tscorestream;
		map <string, double>::iterator itr;
		for (itr = advisorTScore.begin(); itr != advisorTScore.end(); itr++) {
			tscorestream << itr->second << " ";
		}
		
		stringstream difftscoresstream;
		for (size_t i=0; i < diffTScores.size(); i++) {
			difftscoresstream << diffTScores[i] << " ";
		}
		
		stringstream diffoverallsupportsstream;
		for (size_t i=0; i < diffOverallSupports.size(); i++) {
			diffoverallsupportsstream << diffOverallSupports[i] << " ";
		}
		
		stringstream output;
		output << atof(vstrings[0].c_str()) << "\t" << atof(vstrings[1].c_str()) << "\t" << atof(vstrings[2].c_str()) << "\t" << decisionTier << "\t" << computationTimeSec << "\t" << tscorestream.str() << "\t" << gini << "\t" << overallSupport << "\t" << confidenceLevel << "\t" << difftscoresstream.str() << "\t" << diffoverallsupportsstream.str();
		
		logData.data = output.str();
		explanations_log_pub_->publish(logData);
	}
};

// Main file : Load configuration file

int main(int argc, char **argv) {

	rclcpp::init(argc, argv);
	try {
		auto explain = make_shared<Explanation>();
		const string default_config =
			ament_index_cpp::get_package_share_directory("why") +
			"/config/text.conf";
		explain->declare_parameter<string>("text_config", default_config);
		explain->initialize(
			explain->get_parameter("text_config").as_string());
		RCLCPP_INFO(explain->get_logger(), "Explanation initialized");
		explain->run();
		if(rclcpp::ok()){
			rclcpp::shutdown();
		}
		return 0;
	} catch (const exception & error) {
		if(!rclcpp::ok()){
			return 0;
		}
		auto logger = rclcpp::get_logger("why");
		RCLCPP_FATAL(logger, "Failed to start explanation node: %s", error.what());
		rclcpp::shutdown();
		return 1;
	}
}
