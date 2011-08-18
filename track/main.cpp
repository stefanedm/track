//#include <QtCore/QCoreApplication>

//#include <OpenSG/OSGQT4WindowBase.h>
#include <OpenSG/OSGConfig.h>
#include <OpenSG/OSGGLUT.h>
#include <OpenSG/OSGGLUTWindow.h>
#include <OpenSG/OSGSimpleGeometry.h>
#include <OpenSG/OSGSimpleSceneManager.h>



#include <iostream>

OSG_USING_NAMESPACE

OSG::SimpleSceneManager* mgr;
using namespace std;
int setupGLUT(int *argc, char *argv[]);

int main(int argc, char *argv[])
{
  //  QCoreApplication a(argc, argv);

    OSG::osgInit(argc,argv);
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

    //return a.exec();
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

   // glutInitDisplayMode(GLUT_RGB);

    int winid = glutCreateWindow("OpenSG First Application");

    // register the GLUT callback functions
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);

    return winid;
}
