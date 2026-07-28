/*
 * This is header file for tier-1 advisors. 
 *
 * Date Created: Jan. 7, 2014.
 * Last Edited: Jan. 7, 2014.
 * Created by: Slavisa Djukic <sdjukic@hunter.cuny.edu>
 */

#include <string>
#include <vector>
#include <semaforr/core/FORRAction.h>
#include <semaforr/decision/Beliefs.h>
#include <semaforr/exploration/LocalExplore.h>

// A t1 advisor class contains a list of functions each of which returns a single action or vetoes a set of actions
class Tier1Advisor {

public:
	Tier1Advisor(Beliefs *b){
		beliefs = b;
		shortcut = false;
	}

	void advisorNotOpposite();

	bool advisorAvoidObstacles();

	bool advisorDontGoBack();

	bool advisorGetOut(FORRAction *decision);

	bool advisorVictory(FORRAction *decision);

	bool advisorEnforcer(FORRAction *decision);

	bool advisorDoorway(FORRAction *decision);

	bool advisorFindAWay(FORRAction *decision);

	bool advisorBehindYou(FORRAction *decision);

	void resetLocalExploration(){
		localExploration.resetLocalExplorer();
	}

	bool localExplorationStarted(){
		return localExploration.getAlreadyStarted();
	}

	bool localExplorationTriggerLearning(){
		if(localExploration.getAlreadyStarted()){
			return localExploration.triggerLearning();
		}
		else{
			return false;
		}
	}

	vector< vector<int> > getLocalExploreCoverage(){
		if(localExploration.getAlreadyStarted()){
			return localExploration.getCoverage();
		}
		else{
			return vector< vector<int> >();
		}
	}

	bool getShortcut(){
		return shortcut;
	}

	void resetShortcut(){
		shortcut = false;
	}
	
private:
	Beliefs *beliefs;
	LocalExplorer localExploration;
	bool shortcut;
};
