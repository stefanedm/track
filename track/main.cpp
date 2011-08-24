//#include <QtCore/QCoreApplication>


// OPENSG INCLUDES
//#include <OpenSG/OSGQT4WindowBase.h>
#include <OpenSG/OSGConfig.h>
#include <OpenSG/OSGGLUT.h>
#include <OpenSG/OSGGLUTWindow.h>
#include <OpenSG/OSGSimpleGeometry.h>
#include <OpenSG/OSGSimpleSceneManager.h>

// OPENCV INCLUDES
#include <opencv2/opencv.hpp>

// DEBUG & OUTPUT
#include <QDir>
#include <iostream>

// ARToolKit INCLUDES
#include <AR/gsub.h>
#include <AR/video.h>
#include <AR/param.h>
#include <AR/ar.h>

#include <GLUT/glut.h>
#include <OpenGL/gl.h>

#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <string>

OSG_USING_NAMESPACE

OSG::SimpleSceneManager* mgr;
using namespace std;
int setupGLUT(int *argc, char *argv[]);
char* itoa( int value, char* result, int base );

int main(int argc, char *argv[])
{
    int j = 0;

    // Allocate Sotrage
    std::vector<std::vector<cv::Point3f> > object_points;
    std::vector<std::vector<cv::Point2f> > image_points;

    int numBoards = 40;
    int numCornersHor = 9;
    int numCornersVer = 6;

    int numSquares = numCornersHor * numCornersVer;
    cv::Size board_sz = cv::Size(numCornersVer, numCornersHor);

    vector<cv::Point2f> corners;
    int successes=0;


    cv::Mat init_Image;

	vector<cv::Point3f> obj;
	for(int j=0;j<numSquares;j++)
	    obj.push_back(cv::Point3f(j/numCornersHor, j%numCornersHor, 0.0f));

	//while(successes<numBoards)
	for(int i = 1 ; i< 241 ; i++){
	    // every 10 pictures
	    if(j != 5){
		j++;
		continue;
	    } else {
		j=0;
	    }
	    stringstream stream;
	    stream << "pics/init/";
	    stream << i;
	    stream << ".bmp";
	    cout << "#### PICTURE NO. "<<i<<"####"<<endl;
	    init_Image = cvLoadImage(stream.str().c_str());
	    cv::Mat grayImage;

	    cv::cvtColor(init_Image, grayImage, CV_BGR2GRAY);
	    bool found = cv::findChessboardCorners(init_Image, board_sz, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

	   if(found)
	   {
	       cv::cornerSubPix(grayImage, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
	       cv::drawChessboardCorners(grayImage, board_sz, corners, found);
	   }

	   cv::imshow("Calib1",init_Image);
	   cv::imshow("Calib2",grayImage);

	   int key = cv::waitKey(1);
	   if(key==27)
	       return 0;

	    if(found!=0)
	    {
		image_points.push_back(corners);
		object_points.push_back(obj);
		printf("Snap stored!\n");

		successes++;

		if(successes>=numBoards)
		    break;
	    }

	}

	cv::Mat intrinsic = cv::Mat(3, 3, CV_32FC1);
	cv::Mat distCoeffs;
	vector<cv::Mat> rvecs;
	vector<cv::Mat> tvecs;

	intrinsic.ptr<float>(0)[0] = 1.0;
	intrinsic.ptr<float>(1)[1] = 1.0;

	intrinsic.ptr<float>(0)[2] = 1.0;
	intrinsic.ptr<float>(1)[2] = 1.0;

	cout << "SIZE:: " << init_Image.size().height << " _ " << init_Image.size().width<<endl;

	calibrateCamera(object_points, image_points, init_Image.size(), intrinsic, distCoeffs, rvecs, tvecs,CV_CALIB_USE_INTRINSIC_GUESS);

	cout << "##INTRINSIC##" <<endl;
	cout << intrinsic.ptr<float>(0)[0] << " " <<intrinsic.ptr<float>(0)[1]<<" " <<intrinsic.ptr<float>(0)[2]<< endl;
	cout << intrinsic.ptr<float>(1)[0] << " " <<intrinsic.ptr<float>(1)[1]<< " " <<intrinsic.ptr<float>(1)[2] << endl;
	cout << intrinsic.ptr<float>(2)[0] << " " <<intrinsic.ptr<float>(2)[1]<< " " <<intrinsic.ptr<float>(2)[2] << endl;

	//cvSave( "Intrinsics.xml", intrinsic.ptr() );

	// Save the intrinsics and distortions
	//cv::
	//cv::Save( "Intrinsics.xml", intrinsic );
	//cv::Save( "Distortion.xml", distCoeffs);






    return 0;
    // ar test

    char           *cparam_name    = "pics/camera_para.dat";
    ARParam         cparam;
    ARParam  wparam;

   // return 0;
    // opencv-test
    IplImage* frame;

    IplImage* grey = NULL;

    IplImage* edges = NULL;

    char    *patt_name      = "davit.patt";

    int key(0);

    cv::VideoCapture capture = 0;
    std::cout << QDir::currentPath().toStdString() << endl;
    if(!capture.open("test.avi"))
	return 1;
    cv::Mat m,n;
    cv::namedWindow("looky looky");

    ARUint8  *dataPtr;

    int patt_id;
    if( (patt_id=arLoadPatt(patt_name)) < 0 ) {
	cout << "Pattern load error" <<endl;
    }

    /* set the initial camera parameters */
    if( arParamLoad(cparam_name, 1, &wparam) < 0 ) {
	printf("Camera parameter load error !!\n");
	exit(0);
    }
    arParamChangeSize( &wparam, 640, 480, &cparam );
    arInitCparam( &cparam );
    printf("*** Camera Parameter ***\n");
    arParamDisp( &cparam );

    //while(capture.grab()){
	//capture.retrieve(m);
    for(int u=1;u<=100;++u){


	frame = cvLoadImage("pics/test",3);
	cv::Mat RGB(frame);
	//cv::Mat RGB (m);
	cv::Mat RGBA (RGB.rows,RGB.cols,CV_8UC4);
	//cv::flip(RGB,RGB,0);

	cv::Mat ABGR(RGBA.rows,RGBA.cols, CV_8UC4);
	cv::Mat alpha(RGB.rows,RGB.cols,CV_8UC1);
	//convert RGBA to ABGR
	cv::Mat in[] = {RGB,alpha};
	cv::Mat out[] = {ABGR};
	// RGBA[0] --> ABGR[3]
	// RGBA[1] --> ABGR[2]
	// RGBA[2] --> ABGR[1]
	// RGBA[3] --> ARGB[0]
	int from_to[] = {0,1,1,2,2,3,3,0};
	//BGR = RGBA
	cv::mixChannels(in,2,out,1,from_to,4);

	ARMarkerInfo    *marker_info;
	int             marker_num;

	if( (dataPtr = (ARUint8 *)ABGR.data) == NULL ) {
		arUtilSleep(2);
		cout << "No Image data found...";
		return 2;
	}

	/*ARUint8 *dataPtr = (ARUint8*) malloc(sizeof(ARUint8)*ABGR.cols*ABGR.rows*4);
	int i = 0;
	for(int y = 0;y < ABGR.rows;y++) {
	    for(int x = 0; x < ABGR.cols; x++) {
		for(int j = 0; j < 4;j++){
		   // cout << "." << i;
		    dataPtr[i++]  = ABGR.data[y*(ABGR.step/sizeof(uchar))+x*4+j];
		}
	    }
	}
	cout <<"ENDE"<<endl;
*/

	if( arDetectMarker(dataPtr, 100, &marker_info, &marker_num) < 0 ) {
	    cout << "No Markers found... :(";
		//cleanup();
		//exit(0);
		return 4;
	}

	cout << "Marker found : " << marker_num << endl;

	int j,k;
	k = -1;
	    for( j = 0; j < marker_num; j++ ) {
		if( patt_id == marker_info[j].id ) {
		    if( k == -1 ) k = j;
		    else if( marker_info[k].cf < marker_info[j].cf ) k = j;
		}
	    }
	    cout << "k: " << k << endl;

	cv::imshow("looky looky",ABGR);
	cv::waitKey(1);
	}
    //}
/*
	if(!capture){

	    return 1;

	} else {
	    cout << "Got capture... \n";
	    //return 2;
	}

	int fps = (int)cvGetCaptureProperty(capture,CV_CAP_PROP_FPS);

	cout << "fps: " << fps << "n";

	cvNamedWindow("video", 0);

	cout << "created window... \n";

	while(key != 'q') {
	    cout << "in loop... \n";

	    frame = cvQueryFrame(capture);

	    //cv::Mat tmp(frame);

	    //cv::Mat tmp1(grey);

	    cv::flip(frame, frame, 0);

	    // start tracking

	    //grey = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);

	    //edges = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);

	    //cv::cvtColor(tmp,tmp1,CV_RGB2GRAY,0);

	    //cvCvtColor(frame, grey, CV_BGR2GRAY);

	    //cvCanny( grey, edges, 1.0, 1.0, 3);

	    // finish

	    if(!frame) break;

	    cvShowImage("video",frame);

	    key = cvWaitKey(1000/fps);

	}
	cout << "SHUT DOWN EVERYTHING!!!!!";

	cvReleaseCapture(&capture);

	cvDestroyWindow("video");

*/


  //  QCoreApplication a(argc, argv);

   /* OSG::osgInit(argc,argv);
    {
	int winid = setupGLUT(&argc,argv);
	OSG::GLUTWindowRecPtr gwin = OSG::GLUTWindow::create();
	gwin->setGlutId(winid);
	gwin->init();

	// This will be our whole scene for now : an incredible torus
	OSG::NodeRecPtr scene = OSG::makeTorus(.5, 2, 16, 16);

	// Create and setup our little friend - the SSM
	mgr = new OSG::SimpleSceneManager;
	mgr->setWindow(gwin);
	mgr->setRoot(scene);
	mgr->showAll();


	cout<<"wooooaaaaaahhh";
    }
    glutMainLoop();

    //return a.exec();*/
    return 0;
}

// react to size changes
void reshape(int w, int h)
{
    mgr->resize(w, h);
    glutPostRedisplay();
}

// just redraw our scene if this GLUT callback is invoked
void display(void)
{
    mgr->redraw();
}

int setupGLUT(int *argc, char *argv[])
{
    glutInit(argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);

    int winid = glutCreateWindow("OpenSG First Application");

    // register the GLUT callback functions
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);

    return winid;
}

/**

 * C++ version char* style "itoa":

 */

char* itoa( int value, char* result, int base ) {

	// check that the base if valid

	if (base < 2 || base > 16) { *result = 0; return result; }



	char* out = result;

	int quotient = value;



	do {

		*out = "0123456789abcdef"[ std::abs( quotient % base ) ];

		++out;

		quotient /= base;

	} while ( quotient );



	// Only apply negative sign for base 10

	if ( value < 0 && base == 10) *out++ = '-';



	std::reverse( result, out );

	*out = 0;

	return result;

}

