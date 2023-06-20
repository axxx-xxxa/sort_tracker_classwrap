#include <iostream>
#include <fstream>
#include <iomanip>
#include <set>
#include "Hungarian.h"
#include "KalmanTracker.h"

typedef struct TrackingBox
{
	int frame;
	int id;
	Rect_<float> box;
}TrackingBox;


// Computes IOU between two bounding boxes
double GetIOU(Rect_<float> bb_test, Rect_<float> bb_gt);

void VecTest(string seqName, std::vector<TrackingBox>& detData);

void TestSORT(std::vector<TrackingBox>& detData, bool display);


class Sort {
public:
	int init();
	int update_Vectest(std::vector<TrackingBox>& detData, std::ofstream& resultsFile);



protected:

	// recorders
	int maxFrame		= 0;
	int frame_count		= 0;
	int total_frames	= 0;
	float total_time	= 0;
	double cycle_time	= 0.0;
	int64 start_time	= 0;
	unsigned int trkNum = 0;
	unsigned int detNum = 0;

	// params
	int max_age = 1;
	int min_hits = 3;
	double iouThreshold = 0.3;

	// tracker staff
	set<int> allItems;
	set<int> matchedItems;
	set<int> unmatchedDetections;
	set<int> unmatchedTrajectories;
	vector<int> assignment;
	vector<cv::Point> matchedPairs;
	vector<KalmanTracker> trackers;
	vector<vector<double>> iouMatrix;
	vector<Rect_<float>> predictedBoxes;
	vector<TrackingBox> frameTrackingResult; 
	

	
	//vector<vector<TrackingBox>> detFrameData;
	//vector<TrackingBox> tempVec;
private:

};