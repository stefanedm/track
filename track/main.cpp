#include <QtCore/QCoreApplication>

#include <map>

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
vector<Pnt3f> createControlPoints(Pnt3f p1, Pnt3f p2);
void sortLines();

GeometryPtr TEST;
//angle of rotation
float xpos = 0, ypos = 0, zpos = 0, xrot = 0, yrot = 0, angle=0.0;

vector<Pnt3f> _CONTROLPOINTS;
vector<Pnt3f> _LINES;

int main(int argc, char *argv[])
{

    /*int j = 0;

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
*/
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

    arParamChangeSize( &wparam, 1920, 1080, &cparam );
    arInitCparam( &cparam );
    cout << "*** Camera Parameter ***"<<endl;
    arParamDisp( &cparam );

    //open file
    fstream file;
    file.open("1920x1080Data.dat",ios::out);

    cv::Mat RGB;
    int v = 0;
    for(int u=1;u<=520;++u){
	file<<"#FRAME_"<<u<<endl;
	//cout << "in loop..."<<endl;
	stringstream stream;
	stream << "pics/";
	stream << u;
	stream << ".bmp";

	RGB = cv::imread(stream.str().c_str());
	//cv::flip(RGB,RGB,0);

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
	int winid = setupGLUT(&argc,argv);
	//GLUTWindowPtr gwin = GLUTWindow::create();
	//gwin->setGlutId(winid);
	//gwin->init();


	OSG::NodePtr scene = SceneFileHandler::the().read("data/test3_3.obj");
	//GroupPtr scene = GroupPtr::dcast(scene);


	cout << "type: " << scene.getCore()->getTypeName()<< endl;

	cout << "children in scene: " << scene->getNChildren()<<endl;

	//for(int i(0);i < scene->getNChildren();++i)
	//{
/*		cout << "child no. "<<i<<endl;
		cout << "type of child: "<<scene->getChild(i)->getTypeName()<<endl;
		NodePtr child = scene->getChild(i);
		cout << "children of child: " << child->getNChildren()<<endl;
		cout << "type: " << child.getCore()->getTypeName()<<endl;
*/		/*NodePtr child2 = child->getChild(0);
		cout << "children of child2: " << child2->getNChildren()<<endl;
		cout << "type: " << child2.getCore()->getTypeName()<<endl;
		NodePtr child3 = child2->getChild(0);
		cout << "children of child3: " << child3->getNChildren()<<endl;
		cout << "type: " << child3.getCore()->getTypeName()<<endl;
		NodePtr child4 = child3->getChild(0);
		cout << "children of child4: " << child4->getNChildren()<<endl;
		cout << "type: " << child4.getCore()->getTypeName()<<endl;
*/
		//NodePtr geo = child4;
//		GeometryPtr geo = GeometryPtr::dcast(child->getCore());
		GeometryPtr geo = GeometryPtr::dcast(scene->getCore());

		FaceIterator it;

//		GeometryPtr geom = Geometry::create();
		GeoPositions3fPtr pos = GeoPositions3f::create();
		GeoNormals3fPtr norms = GeoNormals3f::create();
		GeoPTypesPtr type = GeoPTypesUI8::create();
		type->addValue(GL_LINE);

		LineIterator lit;
		int lines(0);
		TEST = geo;
		for(lit = geo->beginLines();lit != geo->endLines();++lit){
			lines++;
			//cout << "pos1: " << lit.getPosition(0)<<endl;
			//cout << "pos2: " << lit.getPosition(1)<<endl;
			//cout << "wtf ?! "<<(lit.getPosition(0)).dist(lit.getPosition(1))<<endl;
			if((lit.getPosition(0)).dist(lit.getPosition(1)) > 1){


				cout << "GROßES BADABOOOM! >> "<<(lit.getPosition(0)).dist(lit.getPosition(1))<< endl;
			}
		}

		cout << "lines: " << lines <<endl;
		SimpleMaterialPtr mat = SimpleMaterial::create();
		geo->setMaterial(mat);

	//}
	//NodePtr	root = calcVertexNormalsGeo(geom, 1.0);
		//Matrix m;
		//m.setIdentity();

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

	gluPerspective (60, (GLfloat)w / (GLfloat)h, 0.1, 30.0); //set the perspective (angle of sight, width, height, ,depth)

	float m[16] = {0.9132,0.3692,-0.1720,0,
		      -0.3475,0.9265,0.1437,0,
		      0.2123,-0.0714,0.9745,0,
		      0.225,-0.277,-10,1};

	float m_ori[16] = {	0.9132,-0.3692,-0.1720,0,
				-0.3475,-0.9265,0.1437,0,
				-0.2123,-0.0714,-0.9745,0,
				0.225,0.277,-10,1};

	glMultMatrixf(m);
	//glTranslatef(1,-4.5,0);
	glMatrixMode (GL_MODELVIEW); //set the matrix back to model
	glLoadIdentity();

	glScalef(1.4,1.4,1.4);

}

// just redraw our scene if this GLUT callback is invoked
void display(void)
{
    mgr->redraw();
}

void renderScene(void) {
	int window_w = 960;
	int window_h = 540;

	cout << "render scene... "<<endl;

	TriangleIterator ti;
	glEnable(GL_DEPTH_TEST);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glBegin(GL_TRIANGLES);

	//map<int, vector<Pnt3f> > _COLORS;
	map<int, int> _COLORS;

	for(ti = TEST->beginTriangles();ti != TEST->endTriangles();++ti)
	{
		int rgb_index = 0;
		int r,g,b;
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
						glVertex3f(fit.getPosition(0)[0],fit.getPosition(0)[1],fit.getPosition(0)[2]);
						glVertex3f(fit.getPosition(1)[0],fit.getPosition(1)[1],fit.getPosition(1)[2]);
						_LINES.push_back(fit.getPosition(0));
						_LINES.push_back(fit.getPosition(1));
						//createControlPoints(fit.getPosition(0),fit.getPosition(1));
					}
					h++;
				}
			// a-c
			if(a == a2 || a == b2 || a == c2)
				if(c == a2 || c == b2 || c == c2){
					if(fit.getNormal(0).dot(nit.getNormal(0)) < thresh){
						count_lines_drawn++;
						glVertex3f(fit.getPosition(0)[0],fit.getPosition(0)[1],fit.getPosition(0)[2]);
						glVertex3f(fit.getPosition(2)[0],fit.getPosition(2)[1],fit.getPosition(2)[2]);
						_LINES.push_back(fit.getPosition(0));
						_LINES.push_back(fit.getPosition(2));
						//createControlPoints(fit.getPosition(0),fit.getPosition(2));
					}
					h++;
				}
			// c-b
			if(c == a2 || c == b2 || c == c2)
				if(b == a2 || b == b2 || b == c2){
					if(fit.getNormal(0).dot(nit.getNormal(0)) < thresh){
						count_lines_drawn++;
						glVertex3f(fit.getPosition(1)[0],fit.getPosition(1)[1],fit.getPosition(1)[2]);
						glVertex3f(fit.getPosition(2)[0],fit.getPosition(2)[1],fit.getPosition(2)[2]);
						_LINES.push_back(fit.getPosition(1));
						_LINES.push_back(fit.getPosition(2));
						//createControlPoints(fit.getPosition(1),fit.getPosition(2));
					}
					h++;
				}
		}

		glEnd();
	}

	//glutSwapBuffers();

	// DRAW _CONTROLPOINTS_

	//sorted lines by length in _LINES
	sortLines();
	// create CP's now
	_CONTROLPOINTS.clear();
	for(int i(0);i < _LINES.size()/10;i=i+2){
		createControlPoints(_LINES[i],_LINES[i+1]);
	}

	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glBegin(GL_POINTS);
	glColor3f(0,1,0);
	cout << "no of CP: "<< _CONTROLPOINTS.size() <<endl;
	for(int i(0);i < _CONTROLPOINTS.size(); ++i){
		glVertex3f(_CONTROLPOINTS[i][0],_CONTROLPOINTS[i][1],_CONTROLPOINTS[i][2]);
	}
	glEnd();
	glutSwapBuffers();

	cout << "number of same edges " << h <<endl;
	cout << "lines drawn: " << count_lines_drawn << endl;
}

vector<Pnt3f> createControlPoints(Pnt3f p1, Pnt3f p2){
	vector<Pnt3f> result;
	Vec3f vec = p2 - p1;
	// calculate length
	float length = sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);
	if (length < 0.2)
		return result;

	//every 5% of the line
	float stepsize = 0.025;
	for(int i(0);i*stepsize <= 1;i++)
	{
		//cout << " cp at ->"<< p1+i*step*vec;
		Pnt3f np;
		np[0] = p1[0] + (stepsize*i) * vec[0];
		np[1] = p1[1] + (stepsize*i) * vec[1];
		np[2] = p1[2] + (stepsize*i) * vec[2];
		//cout << np[0] << " " << np[1] << " " << np[2] <<endl;
		_CONTROLPOINTS.push_back(np);
	}
	return result;
}

void sortLines(){
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
}


int setupGLUT(int *argc, char *argv[])
{
    glutInit(argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);

    int winid = glutCreateWindow("OpenSG First Application");

    // register the GLUT callback functions
    //glutDisplayFunc(display);
    glutDisplayFunc(renderScene);
    //glutIdleFunc(renderScene);
    glutReshapeFunc(reshape);
    glutReshapeWindow(960,540);

    return winid;
}



