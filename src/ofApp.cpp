#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
	ofSetWindowShape(1920, 1080);
	ofSetFrameRate(30);

	if (!initKinect()) exit();

	colorImage.allocate(colorWidth, colorHeight, OF_IMAGE_COLOR_ALPHA);
}

//--------------------------------------------------------------
void ofApp::update(){
	// color
	IColorFrame* pColorFrame = nullptr;
	HRESULT hResult = pColorReader->AcquireLatestFrame(&pColorFrame);

	if (SUCCEEDED(hResult)) {
		hResult = pColorFrame->CopyConvertedFrameDataToArray(colorHeight * colorWidth * colorBytesPerPixels, colorImage.getPixels(), ColorImageFormat_Rgba);
		colorImage.update();
	}

	//body
	IBodyFrame *pBodyFrame = nullptr;
	hResult = pBodyReader -> AcquireLatestFrame(&pBodyFrame);

	if (SUCCEEDED(hResult)) {
		IBody *pBody[BODY_COUNT] = {0};
		hResult - pBodyFrame -> GetAndRefreshBodyData(BODY_COUNT, pBody);

		if (SUCCEEDED(hResult)) {

			jointList.clear();

			for (int count = 0; count < BODY_COUNT; count++) {

				JointState state;
				state.userNum = count;
				jointList.push_back(state);

				BOOLEAN bTracked = false;
				hResult = pBody [count] -> get_IsTracked(&bTracked);

				if (SUCCEEDED(hResult) && bTracked) {

					Joint joint[JointType::JointType_Count];
					hResult = pBody[count] -> GetJoints(JointType::JointType_Count, joint);

					for (int type = 0; type < JointType_Count; type++) {
						jointList.back().joint[type] = joint[type];
					}

				}
			}
		}

		for (int count = 0; count < BODY_COUNT; count++){
			SafeRelease(pBody[count]);
		}


	}

	//release
	SafeRelease(pColorFrame);
	SafeRelease(pBodyFrame);
}

//--------------------------------------------------------------
void ofApp::draw(){
	ofSetColor(255);
	colorImage.draw(0, 0);

	//��r���镔��
	unsigned int comparingPart = JointType::JointType_SpineBase;
	CameraSpacePoint center = CameraSpacePoint();
	const float distanceThreshold = 1.5f;

	//���[�U�[�̃i���o�[�Ƌ���
	vector<float>distanceList;
	if (distanceList.size() != 0) distanceList.clear();
	vector<float>playerNum;
	if (playerNum.size() != 0) playerNum.clear();


	for (int i = 0; i < jointList.size(); i++) {

		//��r����Ԑڈʒu���ǐՏ�ԂɂȂ���ΑΏۊO
		if (jointList[i].joint[comparingPart].TrackingState == TrackingState::TrackingState_NotTracked) {
			continue;
		}

		//�ǐՏ�ԂȂ�l���Ƃ�
		if (jointList[i].joint[comparingPart].TrackingState == TrackingState::TrackingState_Tracked) {
			ofVec3f camspace = ofVec3f(center.X,center.Y,center.Z);
			ofVec3f nowDistance = ofVec3f(jointList[i].joint[comparingPart].Position.X, jointList[i].joint[comparingPart].Position.Y, jointList[i].joint[comparingPart].Position.Z);

			//���S����̋������߂��l��I��
			float distance = camspace.distance(nowDistance);
			if (distance < distanceThreshold) {


				distanceList.push_back(distance);
				playerNum.push_back(i);

			}

		}

		//�Ō�ɂȂ�����
		if (i == jointList.size() - 1)	{

			cout <<  distanceList.size() << endl;

			float minNum;
			float humanNum = 0;

			if (distanceList.size() != 0) {
				//�������r
				for (int i = 0; i < distanceList.size(); i++) {
					if (i == 0) {
						cout << "�ŏ��̋����̔�r" << endl;

						minNum = distanceList[i];
						humanNum = playerNum[i];
					}else{

						//��r
						if (minNum >= distanceList[i]) {
							minNum = distanceList[i];
							humanNum = i;
						}else{
							
						}

					}
				}

				//�֐ߕ`��
				for (int type = 0; type < JointType_Count; type++) {
					cout << "�`��" << endl;

					ColorSpacePoint colorSpacePoint = { 0 };
					pCoordinateMapper->MapCameraPointToColorSpace( jointList[humanNum].joint[type].Position, &colorSpacePoint );
					int x = static_cast<int>( colorSpacePoint.X );
					int y = static_cast<int>( colorSpacePoint.Y );
					if( ( x >= 0 ) && ( x < colorWidth ) && ( y >= 0 ) && ( y < colorHeight ) ){

						ofSetColor(255,0,0);
						ofCircle(x,y,10);

					}
				}
			}





		}



	}


	/*
	//�Ԑڕ����̕`��
	for (int i = 0; i < jointList.size(); i++) {
	for (int type = 0; type < JointType_Count; type++) {
	ColorSpacePoint colorSpacePoint = { 0 };
	pCoordinateMapper->MapCameraPointToColorSpace( jointList[i].joint[type].Position, &colorSpacePoint );
	int x = static_cast<int>( colorSpacePoint.X );
	int y = static_cast<int>( colorSpacePoint.Y );
	if( ( x >= 0 ) && ( x < colorWidth ) && ( y >= 0 ) && ( y < colorHeight ) ){

	ofSetColor(255,0,0);
	ofCircle(x,y,10);

	}
	}
	}*/

}


//--------------------------------------------------------------
bool ofApp::initKinect() {

	//senspr
	HRESULT hResult = S_OK;
	hResult = GetDefaultKinectSensor(&pSensor);
	if (FAILED(hResult)) {
		std::cerr << "Error : GetDefaultKinectSensor" << std::endl;
		return -1;
	}

	hResult = pSensor->Open();
	if (FAILED(hResult)){
		std::cerr << "Error : IKinectSensor::Open()" << std::endl;
		return -1;
	}

	//color
	hResult = pSensor->get_ColorFrameSource(&pColorSource);
	if (FAILED(hResult)){
		std::cerr << "Error : IKinectSensor::get_ColorFrameSource()" << std::endl;
		return -1;
	}

	hResult = pColorSource->OpenReader(&pColorReader);
	if (FAILED(hResult)){
		std::cerr << "Error : IColorFrameSource::OpenReader()" << std::endl;
		return -1;
	}

	hResult = pColorSource->CreateFrameDescription(ColorImageFormat::ColorImageFormat_Rgba, &colorDescription);
	if (FAILED(hResult)){
		std::cerr << "Error : IColorFrameSource::get_FrameDescription()" << std::endl;
		return -1;
	}
	colorDescription->get_Width(&colorWidth);
	colorDescription->get_Height(&colorHeight);
	colorDescription->get_BytesPerPixel(&colorBytesPerPixels);

	//body
	hResult = pSensor -> get_BodyFrameSource(&pBodySource);
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::get_BodyFrameSource()" << std::endl;
		return -1;
	}
	hResult = pBodySource->OpenReader( &pBodyReader );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IBodyFrameSource::OpenReader()" << std::endl;
		return -1;
	}

	//mapper
	hResult = pSensor->get_CoordinateMapper( &pCoordinateMapper );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::get_CoordinateMapper()" << std::endl;
		return -1;
	}

	return true;
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){

}
