#include <QtCore/QCoreApplication>

#include <map>
#include <time.h>

#include <levmar.h>
#include <Accelerate/Accelerate.h>

// OPENSG INCLUDES
#include <OpenSG/OSGConfig.h>
#include <OpenSG/OSGGLUT.h>
#include <OpenSG/OSGSimpleGeometry.h>
#include <OpenSG/OSGGLUTWindow.h>
#include <OpenSG/OSGSimpleSceneManager.h>
#include <OpenSG/OSGSceneFileHandler.h>
#include <OpenSG/OSGGroup.h>
#include <OpenSG/OSGNode.h>
#include <OpenSG/OSGNodePtr.h>
#include <OpenSG/OSGGeometry.h>
#include <OpenSG/OSGLineIterator.h>
#include <OpenSG/OSGEdgeIterator.h>
#include <OpenSG/OSGFaceIterator.h>
#include <OpenSG/OSGTriangleIterator.h>
#include <OpenSG/OSGVector.h>
#include <OpenSG/OSGComponentTransform.h>
#include <OpenSG/OSGPerspectiveCamera.h>
#include <OpenSG/OSGGeoFunctions.h>
#include <OpenSG/OSGSimpleMaterial.h>
#include <OpenSG/OSGMaterial.h>
#include <OpenSG/OSGGeoPropPtrs.h>
#include <OpenSG/OSGTransform.h>

#include <OpenSG/OSGChunkMaterial.h>
#include <OpenSG/OSGMaterialChunk.h>
#include <OpenSG/OSGPolygonChunk.h>
#include <OpenSG/OSGLineChunk.h>


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
void camera(void);
void reshape(int w, int h);
void renderScene();
vector<Pnt3f> createControlPoints(int lineIndex, Pnt3f p1, Pnt3f p2,int thres);
float createControlPoints2D(int i, Pnt2d p1, Pnt2d p2) ;
void sortLines();
void sortLines3D(int thres);
Pnt2f checkNormal(Pnt2d controlPoint, Pnt2f normal, cv::Mat picture);
bool isOutlier(Pnt2f controlPoint, Pnt2f hitPoint,Pnt2f normal, Pnt2f old_hit);
void mapping(double *p, double *x, int m, int n, void *data);
void drawModel(cv::Mat pic, int r, int g, int b);

GeometryPtr TEST;
//angle of rotation
float xpos = 0, ypos = 0, zpos = 0, xrot = 0, yrot = 0, angle=0.0;

map<int,vector<Pnt3f> >_CONTROLPOINTS;
map<int, vector<Pnt2d> > _CP2D;

map<int,vector<Pnt2f> > _HITPOINTS;
vector<Pnt2d> _NORMALS;
vector<Pnt3f> _LINES;
vector<Pnt3d> _LINES2D;

Pnt2d _FIX = Pnt2d(-1,0);
Pnt2d _FIX2 = Pnt2d(-1,0);

// number of controlpoints
int _CP = 800;

// pixel distance to the next control point
int CHECK_PIXEL = 15;

// initialization matrix
//frame 1
/*GLdouble _m[12] = { 0.892563,0.258688,0.857697,
		    -0.386775,1.00967,-0.475882,
		    -0.307668,0.112639,0.893848,
		    -0.243351,-1.00387,-10.1181
};*/

//frame 10
/*GLdouble _m[12] = {0.56232 ,0.431334,-0.0558146,
		   -0.433028 ,0.910384,-0.179817,
		   -0.398728, 0.10957,0.858766,
		   0.0554073, -1.50217, -10.176
};*/

//frame 20
/*GLdouble _m[12] = {0.766846 ,0.461017,0.446555,
		   -0.374064 ,0.886392,-0.272737,
		   -0.521559, 0.0421068,0.852176,
		   0.10954073, -1.20217, -9.4176
};
*/
//frame 30
/*GLdouble _m[12] = {0.841008 ,0.434622,0.322193,
		   -0.316009 ,0.878002,-0.359514,
		   -0.439139, 0.200538,0.875752,
		   0.20954073, -1.10217, -9.4176
};*/
/*GLdouble _m[12] = {0.733921,0.580986,0.553705,
		   -0.294421,1.01429,-0.261679,
		   -0.395801,0.210083,0.687961,
		   0.207058,-1.23651,-9.67958
};*/

// FRAME 30 - 2
/*GLdouble _m[12] = {0.636422,0.386276,0.318426,
		   -0.238123,0.887736,-0.378292,
		   -0.378582,0.185245,0.868635,
		   0.1751,-1.08458,-9.44856
};*/

/*GLdouble _m[12] = {0.665521,0.529517,0.262015,
-0.24959,0.915267,-0.329653,
-0.417254,0.0456434,0.950157,
0.195404,-1.08756,-9.82492};
*/
// frame 40
/*GLdouble _m[12] = {0.579824,0.349663,0.306409,
		   -0.208275,0.932405,-0.243672,
		   -0.519254,0.180053,0.860116,
		   0.41866,-1.13302,-9.45541
};*/

// frame 50
/*GLdouble _m[12] = {0.622915,0.251767,0.271177,
		   -0.127408,0.967661,-0.254847,
		   -0.507016,0.142846,0.958854,
		   0.528486,-0.982229,-9.47013
};*/
/*GLdouble _m[12] = {
	0.760413,0.210032,0.169259,
	-0.249757,0.926865,-0.333332,
	-0.458418,0.22712,1.03501,
	0.20769,-1.10942,-9.60008
};*/
//frame 60
/*GLdouble _m[12] = {0.662915,0.231767,0.261177,
		   -0.107408,0.9767661,-0.254847,
		   -0.557016,0.132846,0.978854,
		   0.568486,-0.932229,-9.47013
};*/

// frame 60 - 2
/*GLdouble _m[12] = {0.694986,0.126102,0.253316,
		   -0.106859,1.00134,-0.211951,
		   -0.419851,0.159652,0.709319,
		   0.550708,-1.00169,-9.49362
};*/

//frame 70
/*GLdouble _m[12] = { 0.998101, 0.031088, 0.229531,
		    -0.0781641, 1.01973, -0.414162,
		    -0.365936, 0.21531, 0.959819,
		    0.5452, -0.94568, -9.76811
};*/

// frame 80
/*GLdouble _m[12] = {0.696085,0.10039,0.250712,
		   -0.047695,1.04153,-0.112604,
		   -0.422337,0.114725,0.56844,
		   0.724322,-0.96737,-9.53117
};*/

// frame 90
/*GLdouble _m[12] = {0.675858,0.0163957,0.108763,
		   0.00743312,1.07743,-0.123585,
		   -0.438039,0.0804241,0.574065,
		   0.897484,-0.944038,-9.52118
};*/

// frame 100
GLdouble _m[12] = {0.684464,-0.0668559,0.116231,
		   0.0640156,1.07902,-0.133541,
		   -0.442349,0.114661,0.580922,
		   1.00601,-0.937953,-9.51425
};

int main(int argc, char *argv[])
{/*
    int j = 0;

    // Allocate Sotrage
    std::vector<std::vector<cv::Point3f> > object_points;
    std::vector<std::vector<cv::Point2f> > image_points;

    int numBoards = 20;
    int numCornersHor = 9;
    int numCornersVer = 6;

    int numSquares = numCornersHor * numCornersVer;
    cv::Size board_sz = cv::Size(numCornersVer, numCornersHor);

    vector<cv::Point2f> corners;
    int successes=0;


    cv::Mat init_Image;

	vector<cv::Point3f> obj;
	for(int y(0); y < numCornersHor; ++y) {
	    double tmp (y * 60.0);
	    for(int x(0); x < numCornersVer; ++x){
		obj.push_back(cv::Point3f(x * 60.0, tmp, 0));
		cout << "::" <<   x * 60.0 << " - " << tmp << endl;
	    }
	}

	//cv::VideoCapture capture = cv::VideoCapture(0);

	//while(successes < numBoards){
	for(int i = 1 ; i< 518 ; i++){
	    // every 10 pictures
	    if(j != 1){
		j++;
		continue;
	    } else {
		j=0;
	    }
	    stringstream stream;
	    stream << "pics/init/";
	    stream << i;
	    stream << ".bmp";
	    cout << "#### PICTURE NO. "<<i<<"####" << stream.str().c_str() <<endl;
	    init_Image = cvLoadImage(stream.str().c_str());
	    cv::Mat grayImage;
	    //capture >> init_Image;

	    cv::cvtColor(init_Image, grayImage, CV_BGR2GRAY);
	    bool found = cv::findChessboardCorners(grayImage, board_sz, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);

	    if(found)
	    {
		cv::cornerSubPix(grayImage, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
		cv::drawChessboardCorners(grayImage, board_sz, corners, found);

		image_points.push_back(corners);
		object_points.push_back(obj);

		cout << "Snap stored!" << endl;
		successes++;

		if(successes>=numBoards)
		    break;
	    }

	   cv::imshow("Calib1",init_Image);
	   cv::imshow("Calib2",grayImage);

	   int key = cv::waitKey(1);


	}
	cout << "image point size : " << image_points.size() << endl;
	cout << "object point size : " << object_points.size() << endl;

	cv::Mat distCoeffs;
	vector<cv::Mat> rvecs;
	vector<cv::Mat> tvecs;

	cv::Mat intrinsic;

	cv::calibrateCamera(object_points, image_points, init_Image.size(), intrinsic, distCoeffs, rvecs, tvecs);

	cout << "##INTRINSIC##" <<endl;
	cout << intrinsic.ptr<double>(0)[0] << " " <<intrinsic.ptr<double>(0)[1]<<" " <<intrinsic.ptr<double>(0)[2]<< endl;
	cout << intrinsic.ptr<double>(1)[0] << " " <<intrinsic.ptr<double>(1)[1]<< " " <<intrinsic.ptr<double>(1)[2] << endl;
	cout << intrinsic.ptr<double>(2)[0] << " " <<intrinsic.ptr<double>(2)[1]<< " " <<intrinsic.ptr<double>(2)[2] << endl;

	cout << "##DISTORTION##" <<endl;
	cout << distCoeffs.ptr<double>(0)[0] << " " << distCoeffs.ptr<double>(0)[1] << " " << distCoeffs.ptr<double>(0)[2] << " "
		<< distCoeffs.ptr<double>(0)[3] << " " << distCoeffs.ptr<double>(0)[4] << " " << distCoeffs.ptr<double>(0)[5] << " "  <<endl;


	for(int i = 1 ; i< 300 ; i++){

	    stringstream stream,s2;
	    stream << "pics/";
	    stream << i;
	    stream << ".bmp";
	    cout << "#### PICTURE NO. "<<i<<"####" << stream.str().c_str() <<endl;
	    cv:: Mat image = cvLoadImage(stream.str().c_str());
	cv::Mat newI;
		cv::undistort(image,newI,intrinsic,distCoeffs);
		s2 << "pics/undist/";
		s2 << i;
		s2 << ".bmp";
		cv::imwrite(s2.str().c_str(),newI);
	}
	exit(0);*/
    // AR _BEGIN_
/*


    const char    *cparam_name    = "pics/camera_para.dat";
    ARParam cparam;
    ARParam wparam;

    // opencv-test
    char	*patt_name      = "davit.patt";

    std::cout << QDir::currentPath().toStdString() << endl;
    cv::Mat m,n;
    cv::namedWindow("looky looky");

    ARUint8  *dataPtr;
    int patt_id;
    if( (patt_id=arLoadPatt(patt_name)) < 0 ) {
	cout << "Pattern load error" <<endl;
    } else {
	cout << "Pattern loaded... "<<endl;
    }

    // set the initial camera parameters
    if( arParamLoad(cparam_name, 1, &wparam) < 0 ) {
	cout << "Camera parameter load error !!"<<endl;
	exit(0);
    } else {
	cout << "Loaded Camera Parameter !! "<<endl;
    }

    //wparam.xsize = 1280;
    //wparam.ysize = 1024;
    //wparam.mat[0][0] = 1598.0;
    //wparam.mat[1][1] = 1596.15;
    //wparam.mat[0][2] = 555.006;
    //wparam.mat[1][2] = 525.028;

    //wparam.dist_factor[0] = -0.477959;
    //wparam.dist_factor[1] = 1.83017;
    //wparam.dist_factor[2] = -0.0036001;
    //wparam.dist_factor[3] = -0.0001997;
    //cout <<"RESET..."<<endl;

    arParamChangeSize( &wparam, 1280, 1024, &cparam );
    arInitCparam( &cparam );
    cout << "*** Camera Parameter ***"<<endl;
    arParamDisp( &cparam );

    //open file
    fstream file;
    file.open("1280x1024Data.dat",ios::out);

    cv::Mat RGB;
    int v = 0;
    for(int u=1;u<=200;++u){
	file<<"#FRAME_"<<u<<endl;
	//cout << "in loop..."<<endl;
	stringstream stream;
	stream << "pics/undist/";
	stream << u;
	stream << ".bmp";

	RGB = cv::imread(stream.str().c_str());
	cv::flip(RGB,RGB,0);

	cv::Mat matAR(RGB.rows,RGB.cols,CV_8UC4);
	cv::cvtColor(RGB,matAR,CV_BGR2BGRA);

	cv::Mat newAR(RGB.rows,RGB.cols,CV_8UC4);

	cv::Mat in[] = {matAR};
	cv::Mat out[] = {newAR};
	int from[] = {3,0,0,3,1,2,2,1};
	cv::mixChannels(in,1,out,1,from,4);

	dataPtr = (ARUint8*)newAR.data;

	ARMarkerInfo    *marker_info;
	int             marker_num;

	if( dataPtr == NULL ) {
		arUtilSleep(2);
		cout << "No Image data found...";
		return 2;
	}

	cout << "try detecting... " << u <<endl;
	if( arDetectMarker(dataPtr, 45, &marker_info, &marker_num) < 0 ) {
	    cout << "No Markers found... :(";
		//cleanup();
		//exit(0);
		return 4;
	}

	int j,k;
	k = -1;
	    for( j = 0; j < marker_num; j++ ) {
		if( patt_id == marker_info[j].id ) {
		    if( k == -1 ) k = j;
		    else if( marker_info[k].cf < marker_info[j].cf ) k = j;
		}
	    }
	    //cout << "k: " << k << endl;
	    double patt_width = 225.0;
	    double patt_center[2] = {0.0,0.0};
	    double patt_trans[3][4];

	    double quat[4];
	    double pos[3];

	    if(marker_num > 0 && k == 0){
		//cout << "Marker found : " << marker_num << "  " << v++ << " - k: " << k <<endl;
		arGetTransMat(&marker_info[k],patt_center,patt_width,patt_trans);
		if(arUtilMat2QuatPos(patt_trans,quat,pos) == 0){
			cout << "quat worked" <<endl;
			cout << "quat: " << quat[0] << " " << quat[1] << " " << quat[2] << " " << quat[3] << endl;
			cout << "pos: " << pos[0] << " " << pos[1] << " " << pos[2] << endl;
			cout <<endl;
		}

		file << patt_trans[0][0] << " " << patt_trans[0][1] << " " << patt_trans[0][2] << " " << patt_trans[0][3] << " "<<endl;
		file << patt_trans[1][0] << " " << patt_trans[1][1] << " " << patt_trans[1][2] << " " << patt_trans[1][3] << " "<<endl;
		file << patt_trans[2][0] << " " << patt_trans[2][1] << " " << patt_trans[2][2] << " " << patt_trans[2][3] << " "<<endl;

		//file << patt_center[0] << " " << patt_center[1] <<endl;
		//cout << "patt width: " << patt_width <<endl;


		file << marker_info[k].pos[0] << " " << marker_info[k].pos[1] <<endl;

		cout << "p1: " << marker_info[k].vertex[0][0] << " " << marker_info[k].vertex[0][1]<< endl;
		cout << "p2: " << marker_info[k].vertex[1][0] << " " << marker_info[k].vertex[1][1]<< endl;
		cout << "p3: " << marker_info[k].vertex[2][0] << " " << marker_info[k].vertex[2][1]<< endl;
		cout << "p4: " << marker_info[k].vertex[3][0] << " " << marker_info[k].vertex[3][1]<< endl;

		cv::line(newAR,cv::Point(marker_info[k].vertex[0][0],marker_info[k].vertex[0][1]),cv::Point(marker_info[k].vertex[1][0],marker_info[k].vertex[1][1]),CV_RGB(255,0,0));
		cv::line(newAR,cv::Point(marker_info[k].vertex[1][0],marker_info[k].vertex[1][1]),cv::Point(marker_info[k].vertex[2][0],marker_info[k].vertex[2][1]),CV_RGB(255,0,0));
		cv::line(newAR,cv::Point(marker_info[k].vertex[2][0],marker_info[k].vertex[2][1]),cv::Point(marker_info[k].vertex[3][0],marker_info[k].vertex[3][1]),CV_RGB(255,0,0));
		cv::line(newAR,cv::Point(marker_info[k].vertex[3][0],marker_info[k].vertex[3][1]),cv::Point(marker_info[k].vertex[0][0],marker_info[k].vertex[0][1]),CV_RGB(255,0,0));

		cv::line(newAR,cv::Point(marker_info[k].vertex[0][0],marker_info[k].vertex[0][1]),cv::Point(marker_info[k].vertex[2][0],marker_info[k].vertex[2][1]),CV_RGB(255,0,0));
		cv::line(newAR,cv::Point(marker_info[k].vertex[1][0],marker_info[k].vertex[1][1]),cv::Point(marker_info[k].vertex[3][0],marker_info[k].vertex[3][1]),CV_RGB(255,0,0));

		cv::line(RGB,cv::Point(marker_info[k].vertex[0][0],marker_info[k].vertex[0][1]),cv::Point(marker_info[k].vertex[1][0],marker_info[k].vertex[1][1]),CV_RGB(255,0,0));
		cv::line(RGB,cv::Point(marker_info[k].vertex[1][0],marker_info[k].vertex[1][1]),cv::Point(marker_info[k].vertex[2][0],marker_info[k].vertex[2][1]),CV_RGB(255,0,0));
		cv::line(RGB,cv::Point(marker_info[k].vertex[2][0],marker_info[k].vertex[2][1]),cv::Point(marker_info[k].vertex[3][0],marker_info[k].vertex[3][1]),CV_RGB(255,0,0));
		cv::line(RGB,cv::Point(marker_info[k].vertex[3][0],marker_info[k].vertex[3][1]),cv::Point(marker_info[k].vertex[0][0],marker_info[k].vertex[0][1]),CV_RGB(255,0,0));

		cv::line(RGB,cv::Point(marker_info[k].vertex[0][0],marker_info[k].vertex[0][1]),cv::Point(marker_info[k].vertex[2][0],marker_info[k].vertex[2][1]),CV_RGB(255,0,0));
		cv::line(RGB,cv::Point(marker_info[k].vertex[1][0],marker_info[k].vertex[1][1]),cv::Point(marker_info[k].vertex[3][0],marker_info[k].vertex[3][1]),CV_RGB(255,0,0));

		cv::waitKey(1);
	    }
	stringstream stream2;
	stream2 << "pics/new/";
	stream2 << u;
	stream2 << ".bmp";
	//cv::imwrite(stream2.str().c_str(),RGB);

	file<< "##"<<endl;
	cv::imshow("looky looky",RGB);
	cv::waitKey(1);
    }
    file.close();

*/
    // AR _ENDE_ !!
    //}


  //  QCoreApplication a(argc, argv);

	OSG::osgInit(argc,argv);
	cout << "argc " << argc <<endl;
	for(int ar(0);ar<argc;++ar)
		cout << argv[ar] << " ";
	cout << endl;
	int winid = setupGLUT(&argc,argv);
	//GLUTWindowPtr gwin = GLUTWindow::create();
	//gwin->setGlutId(winid);
	//gwin->init();


	OSG::NodePtr scene = SceneFileHandler::the().read("data/test__1.obj");
	//OSG::NodePtr scene = SceneFileHandler::the().read("data/test3_4.obj");
	//GroupPtr scene = GroupPtr::dcast(scene);


	cout << "type: " << scene.getCore()->getTypeName()<< endl;

	cout << "children in scene: " << scene->getNChildren()<<endl;

		GeometryPtr geo = GeometryPtr::dcast(scene->getCore());

		GeoPTypesPtr type = GeoPTypesUI8::create();
		type->addValue(GL_LINE);

		LineIterator lit;
		int lines(0);
		TEST = geo;
		for(lit = geo->beginLines();lit != geo->endLines();++lit){
			lines++;
		}

		cout << "lines: " << lines <<endl;
		SimpleMaterialPtr mat = SimpleMaterial::create();
		geo->setMaterial(mat);

	// Create and setup our little friend - the SSM
	mgr = new SimpleSceneManager;
	//mgr->setWindow(gwin);
	//mgr->setRoot(scene);
	//mgr->showAll();
	//glutCreateWindow("test");

    glutMainLoop();

    return 0;
}

// react to size changes
/*void reshape(int w, int h)
{
    mgr->resize(w, h);
    glutPostRedisplay();
}*/

void reshape (int w, int h)
{
	cout << "reshape ... ["<<w<<","<<h<<"]"<< endl;
	glViewport (0, 0, (GLsizei)w, (GLsizei)h); //set the viewport to the current window specifications

	glMatrixMode (GL_PROJECTION); //set the matrix to projectio
	glLoadIdentity();

	//gluPerspective (52, (GLfloat)w / (GLfloat)h, 0.1, 30.0); //set the perspective (angle of sight, width, height, ,depth)
	//glLoadIdentity();

	//generated
	float m_00[16] = {	1.32222, 1.1274, -0.198695, -0.256828,
			-0.688105, 1.61599, 0.28491, 0.381792,
			-0.709301, 0.226619, -0.898599, -0.887848,
			-0.255447, -3.91252, 9.42976, 9.455

	};



	/*
		0.806114 -0.422132 -0.41471 -143.384
		-0.533122 -0.822216 -0.199353 1906.58
		-0.256828 0.381792 -0.887848 9455.03
	*/


	float tmp_m[16];
	tmp_m[0] = _m[0];
	tmp_m[1] = _m[1];
	tmp_m[2] = _m[2];
	tmp_m[3] = 0;

	tmp_m[4] = _m[3];
	tmp_m[5] = _m[4];
	tmp_m[6] = _m[5];
	tmp_m[7] = 0;

	tmp_m[8] = _m[6];
	tmp_m[9] = _m[7];
	tmp_m[10] = _m[8];
	tmp_m[11] = 0;

	tmp_m[12] = _m[9];
	tmp_m[13] = _m[10];
	tmp_m[14] = _m[11];
	tmp_m[15] = 1;
	/*for(int tmp(0);tmp<12;++tmp)
		tmp_m[tmp] = _m[tmp];
*/
	glMultMatrixf(tmp_m);

	glMatrixMode (GL_MODELVIEW); //set the matrix back to model
	glLoadIdentity();

	//float scalev = 1.92;
	float scalev = 0.6;
	glScalef(scalev,scalev,scalev);

}

// just redraw our scene if this GLUT callback is invoked
void display(void)
{
    mgr->redraw();
}

void renderScene(void) {

	//open file
	fstream file;
	stringstream filename;
	filename << "ProjectionData_"<< time(0)<<".dat";

	file.open(filename.str().c_str(),ios::out);


for(int q(100);q>=50;--q)
{
file << "FRAME NO. "<<q<<endl;
for(int fu(0);fu<2;++fu){
	/*if(_m[0]<1)
		_m[0] += 0.1;
	else
		_m[0] = 0.99;*/
/*
	if(_m[4]<1)
		_m[4] += 0.10;
	else
		_m[4] = 0.99;

	if(_m[8]<1)
		_m[8] += 0.10;
	else
		_m[8] = 0.99;*/
	int window_w = 1280;
	int window_h = 1024;

	cout << "render scene... "<<endl;

	TriangleIterator ti;
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glBegin(GL_TRIANGLES);

	//map<int, vector<Pnt3f> > _COLORS;
	map<int, int> _COLORS;

	for(ti = TEST->beginTriangles();ti != TEST->endTriangles();++ti)
	{
		int rgb_index = 0;
		int r(0),g(0),b(0);
		while(_COLORS[rgb_index] != NULL)
		{
			// create random color
			r = (int)rand() % 255;
			g = (int)rand() % 255;
			b = (int)rand() % 255;

			// calculate index
			rgb_index = 256 * g + 256 * 256 * r + b;
			//cout << rgb_index << endl;
		}

		// SAVE ALL _USED_ COLORS
		vector<Pnt3f> v;
		v.push_back(ti.getPosition(0));
		v.push_back(ti.getPosition(1));
		v.push_back(ti.getPosition(2));
		_COLORS[rgb_index] = ti.getIndex();

		//cout << "r " << r << " g "<<g << " b "<<b<<endl;

		// get points from triangle
		Pnt3f p1 = ti.getPosition(0);
		Pnt3f p2 = ti.getPosition(1);
		Pnt3f p3 = ti.getPosition(2);

		//set color and add vertices
		//glColor3f(r,g,b);
		glColor3ub(r,g,b);
		glVertex3f(p1[0],p1[1],p1[2]);
		glVertex3f(p2[0],p2[1],p2[2]);
		glVertex3f(p3[0],p3[1],p3[2]);

	}
	glEnd();

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(1.0, 1.0);

	glBegin(GL_TRIANGLES);
	// set background color
	glColor3f(0,0,0);
	for(ti = TEST->beginTriangles();ti != TEST->endTriangles();++ti)
	{
		Pnt3f p1 = ti.getPosition(0);
		Pnt3f p2 = ti.getPosition(1);
		Pnt3f p3 = ti.getPosition(2);

		glVertex3f(p1[0],p1[1],p1[2]);
		glVertex3f(p2[0],p2[1],p2[2]);
		glVertex3f(p3[0],p3[1],p3[2]);
	}
	glEnd();

	glDisable(GL_POLYGON_OFFSET_FILL);
	glDisable(GL_DEPTH_TEST);

	//glutSwapBuffers();

	map<int,std::vector<int> > color_map;

	// window size
	int size = window_w*window_h*3;
	// read pixels
	GLubyte *pixels = new GLubyte[size];
	glReadPixels(0 , 0 , window_w , window_h , GL_RGB , GL_UNSIGNED_BYTE , pixels);

	// init RGB and count&debug values
	int red,green,blue;
	int count(0);
	//iterate through pixels
	for(int u(0);u < size;u=u+3){
		// get pixels
		red = pixels[u];
		green = pixels[u+1];
		blue = pixels[u+2];
		// calc unique index
		int index = 256 * green + 256 * 256 * red + blue;
		// ignore black
		if(index == 0 )
			continue;

		// fill RGB vector
		vector<int> ct;
		ct.push_back(red);
		ct.push_back(green);
		ct.push_back(blue);

		// put in map
		color_map[index] = ct;
	}

	cout << "Colors seen in frame: "<< color_map.size()<<endl;

	map<int,vector<int> >::iterator mip;
	// for all _visible_ triangles
	int h(0);
	FaceIterator fit = TEST->beginFaces();
	float thresh = 0.95;
	int count_lines_drawn(0);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINES);

	vector<Pnt3f> cp;
	for(mip = color_map.begin();mip != color_map.end();mip++){
		int face_index = _COLORS[mip->first];
		fit.seek(face_index);

		int a = fit.getPositionIndex(0);
		int b = fit.getPositionIndex(1);
		int c = fit.getPositionIndex(2);

		FaceIterator nit;
		glBegin(GL_LINES);
		glColor3f(1.0,0,0);
		for(nit = TEST->beginFaces();nit != TEST->endFaces();++nit)
		{

			if(fit.getIndex() == nit.getIndex())
				continue;
			int a2 = nit.getPositionIndex(0);
			int b2 = nit.getPositionIndex(1);
			int c2 = nit.getPositionIndex(2);

			// a-b
			if(a == a2 || a == b2 || a == c2)
				if(b == a2 || b == b2 || b == c2){
					if(fit.getNormal(0).dot(nit.getNormal(0)) < thresh){
						count_lines_drawn++;
						//glVertex3f(fit.getPosition(0)[0],fit.getPosition(0)[1],fit.getPosition(0)[2]);
						//glVertex3f(fit.getPosition(1)[0],fit.getPosition(1)[1],fit.getPosition(1)[2]);
						_LINES.push_back(fit.getPosition(0));
						_LINES.push_back(fit.getPosition(1));
						//createControlPoints(_LINES.size()-2,fit.getPosition(0),fit.getPosition(1));
					}
					h++;
				}
			// a-c
			if(a == a2 || a == b2 || a == c2)
				if(c == a2 || c == b2 || c == c2){
					if(fit.getNormal(0).dot(nit.getNormal(0)) < thresh){
						count_lines_drawn++;
						//glVertex3f(fit.getPosition(0)[0],fit.getPosition(0)[1],fit.getPosition(0)[2]);
						//glVertex3f(fit.getPosition(2)[0],fit.getPosition(2)[1],fit.getPosition(2)[2]);
						_LINES.push_back(fit.getPosition(0));
						_LINES.push_back(fit.getPosition(2));
						//createControlPoints(_LINES.size()-2,fit.getPosition(0),fit.getPosition(2));
					}
					h++;
				}
			// c-b
			if(c == a2 || c == b2 || c == c2)
				if(b == a2 || b == b2 || b == c2){
					if(fit.getNormal(0).dot(nit.getNormal(0)) < thresh){
						count_lines_drawn++;
						//glVertex3f(fit.getPosition(1)[0],fit.getPosition(1)[1],fit.getPosition(1)[2]);
						//glVertex3f(fit.getPosition(2)[0],fit.getPosition(2)[1],fit.getPosition(2)[2]);
						_LINES.push_back(fit.getPosition(1));
						_LINES.push_back(fit.getPosition(2));
						//createControlPoints(_LINES.size()-2,fit.getPosition(1),fit.getPosition(2));
					}
					h++;
				}
		}

		glEnd();
	}

	//glutSwapBuffers();

	// DRAW _CONTROLPOINTS_

	//sorted lines by length in _LINES
	//sortLines();
	// create CP's now
	_HITPOINTS.clear();
	_CONTROLPOINTS.clear();
	_CP2D.clear();
	_LINES2D.clear();
	/*for(int i(0);i < _LINES.size()/10;i=i+2){
		createControlPoints(i,_LINES[i],_LINES[i+1]);
	}*/
/*
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glBegin(GL_POINTS);
	glColor3f(0,1,0);

	// for each line
	for(int i(0);i < _CONTROLPOINTS.size(); ++i){
		// get CP-vector
		vector<Pnt3f> cps = _CONTROLPOINTS[i];
		// for every single controlpoint
		for(int j(0);j<cps.size();++j)
			glVertex3f(cps[i][0],cps[i][1],cps[i][2]);
	}
	glEnd();
	glutSwapBuffers();
*/
	cout << "Lines over Threshold :: "<< _CONTROLPOINTS.size() <<endl;
	cout << "Shared Edges :: " << h <<endl;
	cout << "Lines drawn :: " << count_lines_drawn << endl;
	cout << "Lines :: "<< _LINES.size()<<endl;

	//open cv
	stringstream stream;
	stream << "pics/undist/resize/";
	stream << q;
	stream << ".bmp";
	/*"pics/1_1.bmp"*/
	cout << "/*** PICTURE "<< stream.str().c_str() << " ***/"<<endl;

	cv::Mat init_Image2 = cvLoadImage(stream.str().c_str());
	cv::flip(init_Image2,init_Image2,0);

	// overwrite canny with ground truth
	//cv::Mat init_Image2 = cvLoadImage("ground_truth/canny/30c.bmp");

	cv::Mat gray;
	cv::Mat init_Image;// = init_Image2.clone();

	cv::cvtColor(init_Image2,gray,CV_RGB2GRAY);


	cv::Mat gaus;
	cv::GaussianBlur(gray,gaus,cv::Size(3,3),1);
	////cv::Sobel(gray,init_Image,init_Image.type(),1,0,3);
	cv::Canny(gaus,init_Image,5,10,3,true);

	vector<vector<cv::Point> > contours;
	vector<vector<cv::Point> > new_contours;
	cv::findContours(init_Image,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE,cv::Point());

	cv::Mat new_contour = cv::Mat(1024,1280,CV_8UC3);
	// for all contours
	for(int c(0);c<contours.size();++c){
		int size = (contours[c]).size();
		//cout << "size = "<< size <<endl;
		cv::Point start = (contours[c])[0];
		cv::Point end = (contours[c])[size];
		float length = sqrt((start.x-end.x)*(start.x-end.x)+(start.y-end.y)*(start.y-end.y));

		//check contours size
		if(contours[c].size()>20 /* && (size-10 < length < size+10)*/){

			for(int c1(0);c1<contours[c].size();++c1){
				cv::Point tmp = (contours[c])[c1];

				cv::line(new_contour,tmp,tmp,CV_RGB(255,255,255));
			}
		}
		//cout << start.x << " " << start.y<<endl;
		//cout << end.x << " " << end.y<<endl;
		//if(length > 20)
		//cout << "::"<<length<<endl;

	}
	//exit(0);
	cv::Mat cont;
	cv::cvtColor(new_contour,cont,CV_RGB2GRAY);
	init_Image = cont.clone();


	//cv::cvtColor(init_Image,init_Image2,CV_GRAY2RGB);
	//init_Image = init_Image2.clone();

	cv::imshow("gray",init_Image);

	// window size
	size = window_w*window_h*3;
	// read pixels
	GLubyte *pixels2 = new GLubyte[size];
	glReadPixels(0 , 0 , window_w , window_h , GL_RGB , GL_UNSIGNED_BYTE , pixels2);

	cv::Mat ogl = cv::Mat(window_h,window_w,CV_8UC3);
	ogl.data = pixels2;

	cv::flip(ogl,ogl,0);
	//cv::imwrite("test45.bmp",ogl);

	CvSize size2;
	size2.height=1024;
	size2.width=1280;
	cv::Mat result = cv::Mat(size2,CV_8UC3);//= (cv::Mat)cvCreateImage(size2,8,3);

	//MY 2D-POINTS
	GLdouble modelview[16], projection2[16];
	GLint viewport[4];

	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
	glGetDoublev(GL_PROJECTION_MATRIX, projection2);
	glGetIntegerv(GL_VIEWPORT, viewport);

	/*double tx, ty, tz;
	for(int i(0); i < _LINES.size(); ++i)
	{
		Pnt3f tmp = _LINES[i];
		cout << "3d-point:" << tmp<<endl;
		gluProject(tmp[0], tmp[1], tmp[2],
			modelview, projection2, viewport,
			&tx, &ty, &tz);

		//cout <<"x: " <<tx << "y: "<<ty<<endl;
		Pnt3d res;
		res[0] = tx;
		res[1] = ty;
		res[2] = tz;
		cout <<"2d-point"<<res<<endl;
		_LINES2D.push_back(res);
	}*/

	int thres = (int)_LINES.size();//(int)(_LINES.size()/2)*0.15;
	sortLines3D(thres);
	_HITPOINTS = map<int,vector<Pnt2f> >();
	double tx1, ty1, tz1;
	double tx2, ty2, tz2;
	// draw 2D-Lines
	//define threshold for CP creation

	//cout << ">>>>>> "<<thres<<endl;
	cv::flip(init_Image,init_Image,0);
	int ok(0);
	for(int i(0);i<thres;i=i+2){
		ok = 0;

		// get 3D Line endings
		Pnt3d p1 = _LINES[i];
		Pnt3d p2 = _LINES[i+1];

		if(p1.dist(p2)<0.3){
			continue;
		}

		// get 2d points of line endings
		gluProject(p1[0], p1[1], p1[2],
			modelview, projection2, viewport,
			&tx1, &ty1, &tz1);

		gluProject(p2[0], p2[1], p2[2],
			modelview, projection2, viewport,
			&tx2, &ty2, &tz2);

		// create Normals
		Pnt2f lineStart, lineEnd;
		lineStart[0]=tx1;lineStart[1]=ty1;
		lineEnd[0]=tx2;lineEnd[1]=ty2;
		Pnt2f lineNormal = lineStart - lineEnd;
		float l = sqrt(lineNormal[0]*lineNormal[0]+lineNormal[1]*lineNormal[1]);
		Pnt2f normal1,normal2;
		//normalized Normals 1 & 2
		normal1[0]=-lineNormal[1]/l;normal1[1]=lineNormal[0]/l;
		normal2[0]=lineNormal[1]/l;normal2[1]=-lineNormal[0]/l;

		// draw line
		//cv::line(result,cv::Point(tx1,ty1),cv::Point(tx2,ty2),CV_RGB(255,0,0),1);

		//creating 2d controlpoints
		//float t = createControlPoints2D(i,p1,p2);
		//if(i <= thres){
			//vector<Pnt2d> cp = _CP2D[i];
			vector<Pnt3f> cp = _CONTROLPOINTS[i];
			Pnt2f old_hit = Pnt2f(-1,0);
			for(int j(0);j<cp.size();++j){
				//get single 3D-ControlPoint
				Pnt3d tmp = cp[j];
				//cout <<"i: "<<i<<" j: "<< j<<endl;
				// fixpoint check!
				if(i==0){
					if(_FIX[0] == -1){
						double x,y,z;
						gluProject(tmp[0], tmp[1], tmp[2],
							modelview, projection2, viewport,
							&x,&y,&z);
						//Pnt2d tmp2d;
						_FIX[0] = x;
						_FIX[1] = y;

					//	cout << "::: "<< (_HITPOINTS[0]).size()<<endl;
					}
					(_HITPOINTS[0]).push_back(_FIX);
					cv::circle(result,cv::Point(_FIX[0],_FIX[1]),3,CV_RGB(255,255,0),2);
					continue;
				}

				// project to 2D
				double x,y,z;
				gluProject(tmp[0], tmp[1], tmp[2],
					modelview, projection2, viewport,
					&x,&y,&z);
				Pnt2d tmp2d;
				tmp2d[0] = x;
				tmp2d[1] = y;
				// CONTROLPOINTS DRAWING
				//cv::circle(result,cv::Point(x,y),2,CV_RGB(100,100,100),2);

				//Pnt2f n1 = _NORMALS[i];
				//Pnt2f n2 = _NORMALS[i+1];
				Pnt2f hit = checkNormal(tmp2d,normal1,init_Image);
				Pnt2f hit1 = checkNormal(tmp2d,normal2,init_Image);

				//cout << "hit" << hit <<endl;

				bool outlier = isOutlier(tmp2d, hit,normal1,old_hit);
				bool outlier2= isOutlier(tmp2d,hit1,normal1,old_hit);
				old_hit = hit;

				//drawing
				if(outlier && outlier2){
					//ok++;
					(_HITPOINTS[i]).push_back(Pnt2f(-1,0));
					//cv::circle(result,cv::Point(hit[0],hit[1]),2,CV_RGB(0,255,255),3);
					//cv::circle(result,cv::Point(hit1[0],hit1[1]),2,CV_RGB(0,255,255),3);
					continue;
					}
				else {
					ok++;
					if(!outlier && !outlier2){
						if(tmp2d.dist(hit)<tmp2d.dist(hit1)){
							//cv::line(result,cv::Point(tmp2d[0],tmp2d[1]),cv::Point(hit[0],hit[1]),CV_RGB(0,255,0),1);
							(_HITPOINTS[i]).push_back(hit);
							old_hit = hit;
							//cv::circle(result,cv::Point(hit1[0],hit1[1]),2,CV_RGB(0,255,255),3);
						}
						else{
							//cv::line(result,cv::Point(tmp2d[0],tmp2d[1]),cv::Point(hit1[0],hit1[1]),CV_RGB(0,255,0),1);
							(_HITPOINTS[i]).push_back(hit1);
							old_hit = hit1;
							//cv::circle(result,cv::Point(hit[0],hit[1]),2,CV_RGB(0,255,255),3);
						}
					} else if(!outlier){
						(_HITPOINTS[i]).push_back(hit);
						//cv::line(result,cv::Point(tmp2d[0],tmp2d[1]),cv::Point(hit[0],hit[1]),CV_RGB(0,255,0),1);
						old_hit = hit;
						//cv::circle(result,cv::Point(hit1[0],hit1[1]),2,CV_RGB(0,255,255),3);
						}
						else {
						(_HITPOINTS[i]).push_back(hit1);
						//cv::line(result,cv::Point(tmp2d[0],tmp2d[1]),cv::Point(hit1[0],hit1[1]),CV_RGB(0,255,0),1);
						old_hit = hit1;
						//cv::circle(result,cv::Point(hit[0],hit[1]),2,CV_RGB(0,255,255),3);
						}
					//cv::line(result,cv::Point(tmp2d[0],tmp2d[1]),cv::Point(tmp2d[0]+normal1[0],tmp2d[1]+normal1[1]),CV_RGB(0,255,0));
					//cv::line(result,cv::Point(tmp2d[0],tmp2d[1]),cv::Point(tmp2d[0]+normal2[0],tmp2d[1]+normal2[1]),CV_RGB(0,255,0));
				}

			}

			/*vector<Pnt2f> t_vec = _HITPOINTS[i];
			if(t_vec.size() < 5){
				_CONTROLPOINTS[i].clear();
				_HITPOINTS[i].clear();
			}*/
			cout << i<<"::: "<< (_HITPOINTS[i]).size()<<endl;
			int realHP(0);
			for(int g(0);g<_HITPOINTS[i].size();++g){
				if(_HITPOINTS[i][g] != -1)
					realHP++;
			}
			if((int)_CONTROLPOINTS[i].size()*0.2 > realHP)
			{
				_CONTROLPOINTS[i].clear();
				_HITPOINTS[i].clear();
			}
			//cout << "REAL HITPOINTS IN LINE "<< i << " -> "<<realHP<<endl;
			//cout << ":::"<<ok<<endl;
			// clear if not enough hitpoints on line
			if(ok < 3 && i != 0){
				//(_HITPOINTS[i]).clear();
				//cout << "clear hitlist"<< endl;
			}
	}
	int countCP = 0;
	//int thres = (int)(_LINES.size()/2)*0.15;
	for(int i(0);i<thres;i=i+2){
		//cout << "line : "<<i<<endl;
		vector<Pnt3f> tmp = _CONTROLPOINTS[i];
		vector<Pnt2f> tmp2 = _HITPOINTS[i];
		for(int j(0);j<tmp.size();++j){
			countCP++;
			Pnt2f hitp = tmp2[j];
			//cv::circle(result,cv::Point(hitp[0],hitp[1]),2,CV_RGB(255,255,0));
			//cout << "Point "<< tmp2[j]<<endl;
		//	cout << "3D-Point "<< tmp[j]<<endl;
		}
		//exit(1);
	}
	cout << "####CONTROLPOINTS>> "<< countCP<<endl;
	countCP = 0;

	cout << "#### LEV - MAR ###"<<endl;
	for(int v(0);v<12;++v){
		cout << " " << _m[v];
		if(v%3 == 2)
			cout <<endl;
	}
	cout << "================"<<endl;

	int ret;
	//for(int go(0);go <=25	; ++go)
	ret = dlevmar_dif(mapping,_m,NULL,12,_CP,1000,NULL,NULL,NULL,NULL,NULL);
	//ret = dlevmar_dif(mapping,_m,NULL,12,_CP,1000,NULL,NULL,NULL,NULL,NULL);

	for(int v(0);v<12;++v){
		cout << " " << _m[v];
		//file << " " << projection2[v];
		if(v%3 == 2){
			cout <<endl;
			//file <<endl;
		}
	}
	file << _m[0] << "," << _m[1] << "," << _m[2] << endl;
	file << _m[3] << "," << _m[4] << "," << _m[5] << endl;
	file << _m[6] << "," << _m[7] << "," << _m[8]<< endl;
	file << _m[9] << "," << _m[10] << "," << _m[11]<< endl;
	file << endl;

	cout <<endl;
	cout <<"iterated for: "<<ret<<endl;
	cout << "#### LEV - MAR  - ENDE###"<<endl;
	//cout << "== HIT POINTS USED " << _HITPOINTS.size()<<endl;
	drawModel(result,0,0,255);

	//convert gray canny image back to rgb

	//cv::cvtColor(init_Image,init_Image2,CV_GRAY2RGB);

	// generated error image
	//cv::add(original,result,result);
	cv::flip(init_Image2,init_Image2,0);
	cv::add(init_Image2,result,result);
	// flip like hell
	cv::flip(result,result,0);

	// save image
	cv::imshow("showing image",result);
	stringstream ss;
	ss << time(0) << ".bmp";
	cv::imwrite(ss.str(),result);
	_CONTROLPOINTS.clear();
	_HITPOINTS.clear();
	_LINES.clear();
}
file << "=====" <<endl;
}
file.close();
}

void drawModel(cv::Mat pic,int r,int g,int b){
	GLdouble modelview[16], projection2[16];
	GLint viewport[4];

	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
	glGetDoublev(GL_PROJECTION_MATRIX, projection2);
	glGetIntegerv(GL_VIEWPORT, viewport);
	for(int g(0);g<_LINES.size();g=g+2){
		double tx1,ty1,tz1,tx2,ty2,tz2;
		Pnt3d p1 = _LINES[g];
		Pnt3d p2 = _LINES[g+1];
		// get 2d points of line endings
		gluProject(p1[0], p1[1], p1[2],
			modelview, projection2, viewport,
			&tx1, &ty1, &tz1);

		gluProject(p2[0], p2[1], p2[2],
			modelview, projection2, viewport,
			&tx2, &ty2, &tz2);
		// draw line
		cv::line(pic,cv::Point(tx1,ty1),cv::Point(tx2,ty2),CV_RGB(r,g,b));
	}
}

void mapping(double *p, double *x, int m, int n, void *data)
{
	double max_res=-1;
	//test save
	//open file
	fstream file;
	stringstream filename;
	filename << "huber"<< time(0)<<".dat";

	//file.open(filename.str().c_str(),ios::out);


	int k;
	GLdouble modelview[16], projection2[16];
	GLint viewport[4];

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective (52, (GLfloat)1280 / (GLfloat)1024, 0.1, 40.0); //set the perspective (angle of sight, width, height, ,depth)
	float tmp_m[16];
	tmp_m[0] = p[0];
	tmp_m[1] = p[1];
	tmp_m[2] = p[2];
	tmp_m[3] = 0;

	tmp_m[4] = p[3];
	tmp_m[5] = p[4];
	tmp_m[6] = p[5];
	tmp_m[7] = 0;

	tmp_m[8] = p[6];
	tmp_m[9] = p[7];
	tmp_m[10] = p[8];
	tmp_m[11] = 0;

	tmp_m[12] = p[9];
	tmp_m[13] = p[10];
	tmp_m[14] = p[11];
	tmp_m[15] = 1;
	/*for(int t(0);t<12;++t){
		//if(t<=7)
		//	tmp_m[t] = 0.1*p[t];
		//else
			tmp_m[t] = p[t];

	}*/

	glMultMatrixf(tmp_m);
	// load new matrix
	//glMultMatrixd(p);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	//glMultMatrixf(m);
	float scalev = 1.92;
	glScalef(scalev,scalev,scalev);

	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
	glGetDoublev(GL_PROJECTION_MATRIX, projection2);
	glGetIntegerv(GL_VIEWPORT, viewport);

	// project 3d point
	//for ever line

	int thres = (int)(_LINES.size());/*0.1*;*/
	int count(0);
	/*int test = rand()%7;
	while(test != 0 || test!=2 || test !=4||test!=6)
		test = rand()%7;
*/
	for(int i(0);i<thres;i=i+2){

		// get hitpoints by line
		vector<Pnt2f> hitpoints = _HITPOINTS[i];
		/*if(hitpoints.size()>10){
			int t = rand() % 2;
			if(t == 0)
				continue;
		}*/
		if(hitpoints.empty())
			continue;
		// jump over small lines
		//if(hitpoints.size()<5)
		//	continue;

		//for every controlpoint
		vector<Pnt3f> cps = _CONTROLPOINTS[i];
		for(int j(0);j<cps.size();++j){
			Pnt2f hit = hitpoints[j];
			if(hit[0] == -1)
				continue;
			Pnt3f cp = cps[j];

			double x1,y,z;
			gluProject(cp[0], cp[1], cp[2],
				modelview, projection2, viewport,
				&x1,&y,&z);
			Pnt2d tmp2d;
			tmp2d[0] = x1;
			tmp2d[1] = y;

			float length = hit.dist(tmp2d);

			if(i <1){
				length = _FIX.dist(tmp2d);
			}

			if(count > n)
				continue;
			//normal caluclation
			x[count] = (double)length*(double)length;
			//file << (double)length * (double)length<<endl;
			//huber calc
			float c = 0.5;
			//cout << "l:"<<length<<endl;
			/*if(length <= c){
				x[count] = ((double)length * (double)length ) / 2 ;
				file << ((double)length * (double)length ) / 2<<endl;
			} else {
				x[count] = c * (double)length - (c * c) / 2;
				file << c * (double)length - (c * c) / 2 << endl;
			}*/
			count++;
		}
		if(count < n && i>=thres){
			cout << "dammit...not enough points...start over!!"<<endl;
			i = 0;
		}
		hitpoints.clear();
		cps.clear();
	}
	//file.close();
	    //exit(0);
}

bool	isOutlier(Pnt2f controlPoint, Pnt2f hitPoint,Pnt2f lineNormal3D, Pnt2f old_CP){
	if(hitPoint[0] == -1)
		return true;
	Pnt2f vec = controlPoint-hitPoint;
	float length = sqrt(vec[0]*vec[0] + vec[1]*vec[1]);
	//cout << length<<endl;

	int test = 10;
	//if(controlPoint[1] > 240)
	//	test = 15;

	if(length > test /*&& length < 0.1*/){
		//cout << "outlier" <<endl;
		return true;
	}

	if(old_CP[0] != -1){
		Pnt2f lineNormal2D = hitPoint-old_CP;
		float dot = lineNormal2D[0]*lineNormal3D[0]+lineNormal2D[1]*lineNormal3D[1];
		//cout << "cos(dot) : "<<cos(dot)<<endl;
		if(dot < 0)
			dot = dot * -1;
		if(dot > 0.9){

			return true;
		}
	} else {
		return true;
	}
	//cout << "!!! "<<hitPoint.dist(old_CP)<<endl;
	//if(hitPoint.dist(old_CP) <1.1)
	//	return true;


	return false;
}

Pnt2f checkNormal(Pnt2d controlPoint, Pnt2f normal, cv::Mat picture){
	Pnt2f result;
	result[0] = -1;
	//cout << "cp: "<< controlPoint<<endl;
	int x(controlPoint[0]),y(controlPoint[1]);
	float min(MAXFLOAT);
	int i(0);

	while((x > 0 && y >0) && (x<1280 && y < 1024)){
		i++;
		Pnt2f tmp;
		tmp[0]= controlPoint[0] + normal[0] * 0.01 * i;
		tmp[1]=controlPoint[1] + normal[1] * 0.01 * i;
		if(x == (int)tmp[0] && y == (int)tmp[1])
			continue;
		x = (int) tmp[0];
		y = (int) tmp[1];
		//check pixels
		//cout <<" x: "<< x << "y: "<< y <<endl;

		cv::Point p = cv::Point(tmp[0],tmp[1]);
		uchar blue = picture.at<uchar>(p);


		//cout << vec <<endl;
		//continue;
		if(blue == 0)
			continue;
		//cout << i<<" hit something"<<endl;
		//cout << vec <<endl;
		return tmp;
	}
	//cout << "hit: "<<result<<endl;
	return result;
}

float createControlPoints2D(int i, Pnt2d p1, Pnt2d p2)
{
	Vec2f vec = p2 - p1;
	// calculate length
	float length = sqrt(vec[0]*vec[0] + vec[1]*vec[1]);
	//if (length < 1)
	//	return -1;

	//every 2.5% of the line
	float stepsize = 0.05;
	vector<Pnt2d> cp;
	for(int i(0);i*stepsize <= 1;i++)
	{
		//cout << " cp at ->"<< p1+i*step*vec;
		Pnt2f np;
		np[0] = p1[0] + (stepsize*i) * vec[0];
		np[1] = p1[1] + (stepsize*i) * vec[1];
		//cout << np[0] << " " << np[1] <<endl;
		cp.push_back(np);
	}
	// calcnormal
	Pnt2f n1; n1[0] = -vec[1]/length*4;n1[1] = vec[0]/length*4;
	Pnt2f n2; n2[0] = vec[1]/length*4;n2[1] = -vec[0]/length*4;
	_NORMALS.push_back(n1);
	_NORMALS.push_back(n2);

	_CP2D[i] = cp;
	return length;// result;
}

vector<Pnt3f> createControlPoints(int lineIndex, Pnt3f p1, Pnt3f p2, int thres){
	vector<Pnt3f> result;
	Vec3f vec = p2 - p1;

	//cout << "start: "<<p1<<endl;
	//cout << "end: "<<p2<<endl;

	float len3D = p1.dist(p2);
	//cout <<"len3d: "<< len3D<<endl;

	//MY 2D-POINTS
	GLdouble modelview[16], projection2[16];
	GLint viewport[4];

	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
	glGetDoublev(GL_PROJECTION_MATRIX, projection2);
	glGetIntegerv(GL_VIEWPORT, viewport);

	double x,y,z;
	gluProject(p1[0], p1[1], p1[2],
		modelview, projection2, viewport,
		&x,&y,&z);
	Pnt2d tmp2d;
	tmp2d[0] = x;
	tmp2d[1] = y;
	double x1,y1,z1;
	gluProject(p2[0], p2[1], p2[2],
		modelview, projection2, viewport,
		&x1,&y1,&z1);
	Pnt2d tmp2d1;
	tmp2d1[0] = x1;
	tmp2d1[1] = y1;

	// length of 2D edge
	float len2D = tmp2d.dist(tmp2d1);
	float stepsize;

	//if(len2D < 10)
	//	return result;

	if(len2D > 100){
	//cout << "len2d: " << len2D<<endl;
	float perc = CHECK_PIXEL * 100 / len2D;
	//cout << "perc: "<<perc<<endl;

	stepsize = /*len3D */ perc / 100 / thres;}
		else {
			stepsize = 0.5 /thres;
		}
	// calculate length
	//float length = sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);

	//cout << "::: " << length<<endl;
	//if (len3D < 0.1)
	//	return result;

	//every 5% of the line
	//float stepsize = 0.5  / thres;
	//float stepsize = (len3D * 0.05) / perc;
	stepsize = stepsize;
	//cout << "stepsize: " << stepsize<<endl;

	float high = tmp2d[1]-tmp2d1[1];
	//cout << "high: "<<high<<endl;
	if(-10 < high && high < 10 && stepsize < 0.25){
		//return result;
		stepsize = stepsize / 2;
	}
	//float stepsize = random / thres;
	for(int i(1);i*stepsize <= 1;i++)
	{
	//cout << "fuu: "<< i*stepsize<<endl;
		//cout << " cp at ->"<< p1+i*step*vec;
		Pnt3f np;
		np[0] = p1[0] + (stepsize*i) * vec[0];
		np[1] = p1[1] + (stepsize*i) * vec[1];
		np[2] = p1[2] + (stepsize*i) * vec[2];

		// stop - in case of longer line calc
		//if((stepsize*i) >= len3D)
		//	break;

		//cout << np[0] << " " << np[1] << " " << np[2] <<endl;
		//_CONTROLPOINTS.push_back(np);


		if(rand()%3 == 0)
			result.push_back(np);
		//if(thres>7)
		//	result.push_back(np);

		//if(-10 < high < 10 && stepsize >= 0.25 )
		//	result.push_back(np);

	}



	//if(result.size() < 10)
	//	result.clear();

	_CONTROLPOINTS[lineIndex] = result;
	//exit(0);
	return result;
}

void sortLines(){
	map<int, float> lengths;
	map<int,float>::iterator mip;
	vector<Pnt3d> result;

	for(int i(0);i<_LINES2D.size();i=i+2)
	{
		Vec2f vec = _LINES2D[i+1] - _LINES2D[i];
		float length = sqrt(vec[0]*vec[0] + vec[1]*vec[1]);
		lengths[i] = length;
	}

	vector<int> index_vec;
	while(lengths.size() != index_vec.size()){
		float max = -1;
		int max_index;
		for(mip = lengths.begin();mip != lengths.end();mip++ ){
			if(mip->second > max){
				max_index = mip->first;
				max = mip->second;
			}
		}
		result.push_back(_LINES2D[max_index]);
		result.push_back(_LINES2D[max_index+1]);
		index_vec.push_back(max_index);
		lengths[max_index] = -1;
	}
	_LINES2D = result;
}

void createFixPoints(){
	vector<Pnt3f> vec;
	for(int i(0);i<50;++i){
		vec.push_back(Pnt3f(-0.141,-1.263,0.227));
	}
	_CONTROLPOINTS[0] = vec;
	/*for(int i(0);i<50;++i){
		vec.push_back(Pnt3f(-0.141,-1.263,-0.227));
	}
	_CONTROLPOINTS[1] = vec;*/
}

void sortLines3D(int thres){
	map<int, float> lengths;
	map<int,float>::iterator mip;
	vector<Pnt3f> result;

	for(int i(0);i<_LINES.size();i=i+2)
	{
		Vec3f vec = _LINES[i+1] - _LINES[i];
		float length = sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);
		lengths[i] = length;
	}

	vector<int> index_vec;
	while(lengths.size() != index_vec.size()){
		float max = -1;
		int max_index;
		for(mip = lengths.begin();mip != lengths.end();mip++ ){
			if(mip->second > max){
				max_index = mip->first;
				max = mip->second;
			}
		}
		result.push_back(_LINES[max_index]);
		result.push_back(_LINES[max_index+1]);
		index_vec.push_back(max_index);
		lengths[max_index] = -1;
	}
	_LINES = result;
	_CONTROLPOINTS.clear();
	_HITPOINTS.clear();
	createFixPoints();
	int lineNumber = (int)_LINES.size()*0.005;
	cout << "line Threshold to create ControlPoints >> "<< lineNumber<<endl;
	for(int i(2);i<thres/*_LINES.size()*/;i=i+2){
		/*int old_i = i;
		i = rand() % _LINES.size();
		if(i %2 != 0 && i+1<=_LINES.size())
			i++;
		else
			i--;
*/
		if(i<lineNumber){
			createControlPoints(i,_LINES[i],_LINES[i+1],5);
			//createControlPoints(i,_LINES[i],_LINES[i+1],5);
		}
		else if(i < lineNumber*2)
			createControlPoints(i,_LINES[i],_LINES[i+1],3);
		else if(i>_LINES.size()-lineNumber)
			createControlPoints(i,_LINES[i],_LINES[i+1],3);
		else
			createControlPoints(i,_LINES[i],_LINES[i+1],3);
		//i =old_i;
		// check number of CP per line
		//cout << "line "<<i<< " no: " << _CONTROLPOINTS[i].size()<<endl;

		Pnt3f first = _CONTROLPOINTS[i][0];
		Pnt3f last = _CONTROLPOINTS[i][_CONTROLPOINTS[i].size()-1];

		if(first[1]>last[1]){
			//cout << "turn around"<<endl;
			vector<Pnt3f> turn;
			for(int v(_CONTROLPOINTS[i].size());v>=0;--v){
				turn.push_back(_CONTROLPOINTS[i][v]);
			}
			_CONTROLPOINTS[i] = turn;
		}

	}

	//exit(0);
}

int setupGLUT(int *argc, char *argv[])
{
	glutInit(argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
	glutInitWindowSize(1280,1024);


	int winid = glutCreateWindow("OpenSG First Application");

	// register the GLUT callback functions
	//glutDisplayFunc(display);

	glutDisplayFunc(renderScene);
	//glutIdleFunc(renderScene);

	//glutReshapeWindow(640,512);
	glutReshapeFunc(reshape);


	return winid;
}



