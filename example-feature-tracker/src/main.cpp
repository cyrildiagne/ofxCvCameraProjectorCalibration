#include "testApp.h"
#include "ofAppGLFWWindow.h"

int main() {
    
    ofAppGLFWWindow* window = new ofAppGLFWWindow();
    window->setMultiDisplayFullscreen(true);
	ofSetupOpenGL(window, SCREEN_WIDTH+PROJECTOR_WIDTH, max(SCREEN_HEIGHT, PROJECTOR_HEIGHT), OF_FULLSCREEN);
	ofRunApp(new testApp());
}
