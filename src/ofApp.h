#pragma once

#include "ofMain.h"
#include <kinect.h>

template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL){
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

typedef struct  JointState{
	int userNum;
	Joint joint[JointType::JointType_Count];
	
};

class ofApp : public ofBaseApp{

public:
	void setup();
	void update();
	void draw();

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);
	bool initKinect();

	//kinect
	IKinectSensor *pSensor;

	//color
	IColorFrameSource *pColorSource;
	IColorFrameSource *pColorFrameSource;
	IColorFrameReader *pColorReader;
	IFrameDescription* colorDescription;
	int colorWidth, colorHeight;
	unsigned int colorBytesPerPixels;

	//body
	IBodyFrameSource *pBodySource;
	IBodyFrameReader *pBodyReader;
	vector<JointState>jointList;

	//mapper
	ICoordinateMapper *pCoordinateMapper;

	//ofImage
	ofImage colorImage;

};
