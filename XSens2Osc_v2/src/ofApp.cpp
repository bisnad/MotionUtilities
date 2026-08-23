#include "ofApp.h"
#include "dab_xsens_stream_manager.h"
#include "dab_xsens_mocap_skeleton.h"
#include "dab_xsens_2_fbx_manager.h"
#include "dab_xsens_osc_manager.h"
#include "dab_file_io.h"
#include "dab_json_helper.h"

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetVerticalSync(true);
	ofSetFrameRate(300);
	ofEnableAntiAliasing();

	try
	{
		loadConfig(ofToDataPath("config.json"));

		dab::xsens::StreamManager::get().setupUDPReceiver(mOscReceivePort);
		dab::xsens::OscManager::get().addSender(mOscSendAddress, mOscSendPort, mOscSendRate);

		//dab::xsens::OscManager::get().start();
	}
	catch (dab::Exception& e)
	{
		std::cout << e << "\n";
	}

	ofSetBackgroundAuto(false);
	ofSetBackgroundColor(255);
}

//--------------------------------------------------------------
void ofApp::update()
{
	try
	{
		dab::xsens::StreamManager::get().update();
	}
	catch (dab::Exception& e)
	{
		std::cout << e << "\n";
	}
}

//--------------------------------------------------------------
void ofApp::draw()
{
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);

	ofSetColor(20);

	try
	{
		dab::xsens::MocapSkeletonManager& skeletonManager = dab::xsens::MocapSkeletonManager::get();
		const std::vector<unsigned int>& skeletonIds = skeletonManager.skeletonIds();

		std::string skeletonIdsString = "skeleton ids: ";
		for (auto skeletonId : skeletonIds) skeletonIdsString += std::to_string(skeletonId) + " ";

		ofDrawBitmapString(skeletonIdsString, 10, 20);

		// Rest-pose calibration is no longer performed in this program --
		// Xsens/MVN Analyze already anatomically calibrates rot_world
		// relative to the performer's own T-pose/N-pose once, at the start
		// of the capture session (inside MVN Analyze itself), so there is
		// no local calibration state left to display here.

		int nextLineY = 40;

		// Per-skeleton property dump (all skeletons)
		for (auto skeletonId : skeletonIds)
		{
			std::shared_ptr<dab::xsens::MocapSkeleton> skeleton = skeletonManager.skeleton(skeletonId);
			const std::vector<std::string>& propertyNames = skeleton->propertyNames();

			ofDrawBitmapString("--- skeleton " + std::to_string(skeletonId) + " properties ---", 10, nextLineY);
			nextLineY += 20;

			for (int pI = 0; pI < propertyNames.size(); ++pI)
			{
				std::string skeletonPropString = "skeleton property name : ";
				skeletonPropString += propertyNames[pI];

				const std::vector<float>& propertyValues = skeleton->property(propertyNames[pI]);

				skeletonPropString += " values : ";
				skeletonPropString += std::to_string(propertyValues[0]);
				skeletonPropString += " ";
				skeletonPropString += std::to_string(propertyValues[1]);
				skeletonPropString += " ";
				skeletonPropString += std::to_string(propertyValues[2]);

				ofDrawBitmapString(skeletonPropString, 10, nextLineY);
				nextLineY += 20;
			}
		}
	}
	catch (dab::Exception& e)
	{
	}
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {
	// No calibration-related shortcuts anymore -- there is no local
	// rest-pose state left to reset. (Left empty; add other shortcuts here
	// if needed.)
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {

}

void
ofApp::loadConfig(const std::string& pFileName) throw (dab::Exception)
{
	try
	{
		std::string restoreString;
		dab::FileIO::get().read(pFileName, restoreString);

		Json::Reader reader;
		Json::Value restoreData;
		dab::JsonHelper& jsonHelper = dab::JsonHelper::get();

		bool parsingSuccessful = reader.parse(restoreString, restoreData);

		if (parsingSuccessful == false) throw dab::Exception("FILE ERROR: failed to parse config data file " + pFileName, __FILE__, __FUNCTION__, __LINE__);

		mOscReceivePort = jsonHelper.getInt(restoreData, "oscReceivePort");
		mOscSendPort = jsonHelper.getInt(restoreData, "oscSendPort");
		mOscSendAddress = jsonHelper.getString(restoreData, "oscSendAddress");
		mOscSendRate = jsonHelper.getFloat(restoreData, "oscSendRate");
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("JSON ERROR: failed to restore config from file " + pFileName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}
