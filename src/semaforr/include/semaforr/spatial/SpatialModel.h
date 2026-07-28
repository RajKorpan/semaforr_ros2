/*!
 * SpatialModel.h
 *
 * \brief Represents the current spatial model of the environment: Regions, exits, conveyors, etc
 *
 * \author Anoop Aroor
 * \date 11/11/2016 Created
 */

#include <semaforr/spatial/FORRRegionList.h>
#include <semaforr/spatial/FORRTrails.h>
#include <semaforr/spatial/FORRConveyors.h>
#include <semaforr/spatial/FORRDoors.h>
#include <semaforr/spatial/FORRHallways.h>
#include <semaforr/spatial/FORRBarriers.h>

class SpatialModel{

public:
	SpatialModel(double width, double height, double granularity)
		: conveyors(width, height, granularity),
		  hallways(width, height) {}

	FORRRegionList* getRegionList(){return &abstract_map;}
	//FORRTrace* getTrace(){return trace;}
	FORRTrails* getTrails(){return &trails;}
	FORRConveyors* getConveyors(){return &conveyors;}
	FORRDoors* getDoors(){return &doors;}
	FORRHallways* getHallways(){return &hallways;}
	FORRBarriers* getBarriers(){return &barriers;}

private:
	FORRRegionList abstract_map;
	//FORRTrace *trace;
	FORRTrails trails;
	FORRConveyors conveyors;
	FORRDoors doors;
	FORRHallways hallways;
	FORRBarriers barriers;
};
