#include "GLViewFinalProjectCode.h"

#include "WorldList.h" //This is where we place all of our WOs
#include "ManagerOpenGLState.h" //We can change OpenGL State attributes with this
#include "Axes.h" //We can set Axes to on/off with this
#include "PhysicsEngineODE.h"
#include <chrono>

//Different WO used by this module
#include "WO.h"
#include "WOStatic.h"
#include "WOStaticPlane.h"
#include "WOStaticTrimesh.h"
#include "WOTrimesh.h"
#include "WOHumanCyborg.h"
#include "WOHumanCal3DPaladin.h"
#include "WOWayPointSpherical.h"
#include "WOLight.h"
#include "WOSkyBox.h"
#include "WOCar1970sBeater.h"
#include "Camera.h"
#include "CameraStandard.h"
#include "CameraChaseActorSmooth.h"
#include "CameraChaseActorAbsNormal.h"
#include "CameraChaseActorRelNormal.h"
#include "Model.h"
#include "ModelDataShared.h"
#include "ModelMesh.h"
#include "ModelMeshDataShared.h"
#include "ModelMeshSkin.h"
#include "WONVStaticPlane.h"
#include "WONVPhysX.h"
#include "WONVDynSphere.h"
#include "WOImGui.h" //GUI Demos also need to #include "AftrImGuiIncludes.h"
#include "AftrImGuiIncludes.h"
#include "AftrGLRendererBase.h"
#include "irrklang.h"
#include "PxPhysicsAPI.h"
#include "WOPhysxBox.h"
#include "WOTerrainPhysx.h"
#include "WODronePhysx.h"
#include "WORay.h"
#include "PhysxPlane.h"
#include "collisionCallback.h"
#include "AftrUtilities.h"
#include "AftrTimer.h"
#include "boost/timer/timer.hpp"
#include "WOCookingPhysx.h"
#include "WOCameraSink.h"
#include "opencv2/opencv.hpp"
#include "imgui/imgui_impl_opengl3.h"
#include "imgui/imgui_impl_sdl.h"
#include "stb/stb_image.h"
#include "imgui/implot/implot.h"
#include "imgui/implot/implot_internal.h"


#define STB_IMAGE_IMPLEMENTATION
using namespace irrklang;
using namespace physx;
using namespace Aftr;

ISoundEngine* engine = createIrrKlangDevice();

GLViewFinalProjectCode* GLViewFinalProjectCode::New( const std::vector< std::string >& args )
{
   GLViewFinalProjectCode* glv = new GLViewFinalProjectCode( args );
   glv->init( Aftr::GRAVITY, Vector( 0, 0, -1.0f ), "aftr.conf", PHYSICS_ENGINE_TYPE::petODE );
   glv->onCreate();
   return glv;
}


GLViewFinalProjectCode::GLViewFinalProjectCode( const std::vector< std::string >& args ) : GLView( args )
{
    f = PxCreateFoundation(PX_PHYSICS_VERSION, a, e);
    mPvd = PxCreatePvd(*f);
    physx::PxPvdTransport* transport = physx::PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 10);
    mPvd->connect(*transport, physx::PxPvdInstrumentationFlag::eALL);
    p = PxCreatePhysics(PX_PHYSICS_VERSION, *f, PxTolerancesScale(), true, mPvd);
    physx::PxSceneDesc sc(p->getTolerancesScale());
    //sc.gravity = physx::PxVec3(0.0f,0.0f,-9.81f);
    mDispatcher = physx::PxDefaultCpuDispatcherCreate(2);
    sc.cpuDispatcher = mDispatcher;
    sc.filterShader = physx::PxDefaultSimulationFilterShader;
    physx::PxCookingParams params(p->getTolerancesScale());
    mCooking = PxCreateCooking(PX_PHYSICS_VERSION, *f, params);
    s = p->createScene(sc);
}


void GLViewFinalProjectCode::onCreate()
{
   //GLViewFinalProjectCode::onCreate() is invoked after this module's LoadMap() is completed.
   //At this point, all the managers are initialized. That is, the engine is fully initialized.
       if( this->pe != NULL )
   {
      //optionally, change gravity direction and magnitude here
      //The user could load these values from the module's aftr.conf
      this->pe->setGravityNormalizedVector( Vector( 0,0,-1.0f ) );
      this->pe->setGravityScalar( Aftr::GRAVITY );
   }
   this->setActorChaseType( STANDARDEZNAV ); //Default is STANDARDEZNAV mode
   //this->setNumPhysicsStepsPerRender( 0 ); //pause physics engine on start up; will remain paused till set to 1
   this->s->setFlag(physx::PxSceneFlag::eENABLE_ACTIVE_ACTORS, true);
}


GLViewFinalProjectCode::~GLViewFinalProjectCode()
{
   //Implicitly calls GLView::~GLView()
}


void GLViewFinalProjectCode::updateWorld()
{
    GLView::updateWorld(); //Just call the parent's update world first.
    //If you want to add additional functionality, do it after
    //move cloud
    cloud_x++;

    
   
    s->simulate(1.0f / 60.0f);
    s->fetchResults(true);
  
    if (gravity) {
        s->setGravity(physx::PxVec3{ gravityDirection.x * 20, gravityDirection.y * 20, gravityDirection.z * 20 });
    }
    //s->setGravity(physx::PxVec3{ gravityDirection.x * 8, gravityDirection.y * 8, gravityDirection.z * 8 });


    physx::PxU32 numActors = 0;
    physx::PxActor** actors = s->getActiveActors(numActors);
    for (physx::PxU32 i = 0; i < numActors; ++i)
    {
        physx::PxActor* actor = actors[i];
        WOPhysxBox* wo = static_cast<WOPhysxBox*>(actor->userData);
        wo->updatePoseFromPhysx();
        
    }

    otherCamera->setPosition(this->drone->getPosition().x, this->drone->getPosition().y, this->drone->getPosition().z+100);
    if (flip) {
        otherCamera->setActorToWatch(this->target5);
    }
    else {
        otherCamera->setActorToWatch(drone);
    }
    
    //if (this->drone != nullptr) {
    //    if (cam->getLookDirection().x > 0) {
    //        this->drone->setPosition(this->cam->getPosition().x + 15, this->cam->getPosition().y, this->cam->getPosition().z + 1);
    //        sink->setPosition(this->drone->getPosition().x + 30, this->drone->getPosition().y + 20, this->drone->getPosition().z);
    //    }
    //    else {
    //        this->drone->setPosition(this->cam->getPosition().x - 15, this->cam->getPosition().y, this->cam->getPosition().z + 1);
    //        sink->setPosition(this->drone->getPosition().x - 30, this->drone->getPosition().y + 20, this->drone->getPosition().z);
    //    }
    //    //this->cam->setPosition(x, y, z);
    //}
    if (cam->getLookDirection().x > 0) {
        wayPt->setPosition(cam->getLookDirection().x*50, cam->getLookDirection().y * 50, cam->getLookDirection().z * 50);

    }
    else {
        wayPt->setPosition(this->cam->getPosition().x + 15, this->cam->getPosition().y, this->cam->getPosition().z + 1);
    }
    
}


void GLViewFinalProjectCode::onResizeWindow( GLsizei width, GLsizei height )
{
   GLView::onResizeWindow( width, height ); //call parent's resize method.
}


void GLViewFinalProjectCode::onMouseDown( const SDL_MouseButtonEvent& e )
{
   GLView::onMouseDown( e );
}


void GLViewFinalProjectCode::onMouseUp( const SDL_MouseButtonEvent& e )
{
   GLView::onMouseUp( e );
}


void GLViewFinalProjectCode::onMouseMove( const SDL_MouseMotionEvent& e )
{
   GLView::onMouseMove( e );
}


void GLViewFinalProjectCode::onKeyDown( const SDL_KeyboardEvent& key )
{
   GLView::onKeyDown( key );
   if( key.keysym.sym == SDLK_0 )
      this->setNumPhysicsStepsPerRender( 1 );

   if( key.keysym.sym == SDLK_1 )
   {

   }
   if (key.keysym.sym == SDLK_5) {
       std::cout << "\n\nTesting ray against octree... For WO" << this->target->getLabel() << "(woID " << this->target->getID() << ")\n";
       Vector head, tail, contactPt;
       this->ray->getRayHeadAndTail(head, tail);
       std::cout << "Ray Head: " << head.toString() << " Ray Tail: " << tail.toString() << "...\n";
       {
           AftrGeometricTerm result = AftrGeometricTerm::geoUNDEFINED;
           {
               boost::timer::auto_cpu_timer t(std::cout, "Boost Wall Time %w\n");
               result = this->target->getNearestPointWhereLineIntersectsMe(tail, head, contactPt, true, false);
           }
           if (result == AftrGeometricTerm::geoSUCCESS)
               std::cout << "Contact Point is " << contactPt.toString() << "\n";
           else
               std::cout << "No Contact Point was found...\n";
       }
   }
   if (key.keysym.sym == SDLK_2) {
       Vector v = centerOfWorld.toVecS();
       v.normalize();
       this->cam->setCameraAxisOfHorizontalRotationViaMouseMotion(v);
       
   }
   if (key.keysym.sym == SDLK_UP)
   {
       
   
       z++;
       
   }
   if (key.keysym.sym == SDLK_w) {
       
       if (d == nullptr) {
           std::string Car(ManagerEnvironmentConfiguration::getLMM() + "/models/drone.obj");
           d = WODronePhysx::New(p, s, Car, Vector(1, 1, 1), 5.0f);
           worldLst->push_back(d);
       }
       Vector cam_look = this->cam->getLookDirection().normalizeMe();
       this->d->addForces(0, 0, cam_look.z);
       
   }

   if (key.keysym.sym == SDLK_SPACE)
   {
       std::string path(ManagerEnvironmentConfiguration::getSMM() + "/models/cube4x4x4redShinyPlastic_pp.wrl");

       Vector cam_pos = this->cam->getPosition();
       WOPhysxBox* wo = WOPhysxBox::New(p, s, path, Vector(1, 1, 1), drone->getPosition().x, drone->getPosition().y, drone->getPosition().z);
       Vector cam_look = this->cam->getLookDirection().normalizeMe();
       wo->addForces(cam_look.x, cam_look.y, cam_look.z);
       
       worldLst->push_back(wo);
       engine->play3D("t", vec3df(drone->getPosition().x, drone->getPosition().x, drone->getPosition().x));
   }
}


void GLViewFinalProjectCode::onKeyUp( const SDL_KeyboardEvent& key )
{
   GLView::onKeyUp( key );
}


void Aftr::GLViewFinalProjectCode::loadMap()
{
   
    
    auto start_time = std::chrono::high_resolution_clock::now();

    //Sound loading ....
    ISoundSource* heli = engine->addSoundSourceFromFile((ManagerEnvironmentConfiguration::getLMM() + "sounds/heli.mp3").c_str());
    engine->addSoundSourceAlias(heli, "tt");
    

    ISoundSource* siren = engine->addSoundSourceFromFile((ManagerEnvironmentConfiguration::getLMM() + "sounds/siren.mp3").c_str());
    engine->addSoundSourceAlias(siren, "s");
    //tap sound
    ISoundSource* tap = engine->addSoundSourceFromFile((ManagerEnvironmentConfiguration::getLMM() + "sounds/sound5.wav").c_str());
    engine->addSoundSourceAlias(tap, "t");

    this->worldLst = new WorldList(); //WorldList is a 'smart' vector that is used to store WO*'s
    this->actorLst = new WorldList();
    this->netLst = new WorldList();

    ManagerOpenGLState::GL_CLIPPING_PLANE = 3000.0;
    ManagerOpenGLState::GL_NEAR_PLANE = 0.5f;
    ManagerOpenGLState::enableFrustumCulling = false;
    Axes::isVisible = true;
    this->glRenderer->isUsingShadowMapping(false); //set to TRUE to enable shadow mapping, must be using GL 3.2+

    this->cam->setPosition(10, 15, 10);

    std::string shinyRedPlasticCube(ManagerEnvironmentConfiguration::getSMM() + "/models/cube4x4x4redShinyPlastic_pp.wrl");
    std::string wheeledCar(ManagerEnvironmentConfiguration::getLMM() + "/models/drone.obj");
    std::string grass(ManagerEnvironmentConfiguration::getSMM() + "/models/road26x10.wrl");
    std::string human(ManagerEnvironmentConfiguration::getSMM() + "/models/human_chest.wrl");
    //std::string grass(ManagerEnvironmentConfiguration::getSMM() + "/models/road26x10.wrl");


    //WOPhysxBox* wo = WOPhysxBox::New(p, s, shinyRedPlasticCube, Vector(1, 1, 1));
    


    //SkyBox Textures readily available
    std::vector< std::string > skyBoxImageNames; 
  //  skyBoxImageNames.push_back(ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_mountains+6.jpg");
    skyBoxImageNames.push_back(ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_dust+6.jpg");
    

    {
        //Create a light
        float ga = 0.1f; //Global Ambient Light level for this module
        ManagerLight::setGlobalAmbientLight(aftrColor4f(ga, ga, ga, 1.0f));
        WOLight* light = WOLight::New();
        light->isDirectionalLight(true);
        light->setPosition(Vector(0, 0, 2000));
        light->getModel()->setDisplayMatrix(Mat4::rotateIdentityMat({ 0, 1, 0 }, 90.0f * Aftr::DEGtoRAD));
        light->setLabel("Light");
        light_normal = light->getNormalDirection();
        this->targetLight = light;
        lightPos = light->getPosition();
        worldLst->push_back(light);
        
    }
    {
        //PhysxPlane* plane = PhysxPlane::New(p, s, grass, Vector(1, 1, 1));
    }
    
    
    {
        //Create the SkyBox
        WO* wo = WOSkyBox::New(skyBoxImageNames.at(0), this->getCameraPtrPtr());
        wo->setPosition(Vector(0, 0, 0));
        wo->setLabel("Sky Box");
        wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
        worldLst->push_back(wo);
    }
   /* {
        WOCookingPhysx* wo = WOCookingPhysx::New(p, s, mCooking, grass, Vector(5, 1, 1));
        wo->setPosition(62, 0, 0);
        wo->rotateAboutGlobalX(20);
    }*/
    {
        //Camera Sink Implementation
        {
            otherCamera = new CameraChaseActorStationary(this, NULL);
            otherCamera->setPosition(Vector(0, 0, 20));
            
           // otherCamera->setCameraLookAtPoint(Vector(25, 0, 0));
            otherCamera->setCameraLookDirection(Vector(cam->getLookDirection().x, cam->getLookDirection().y, -cam->getLookDirection().z));
            otherCamera->setActorToWatch(drone);
            worldLst->push_back(otherCamera);

            //Camera sink
            sink = WOCameraSink::New(&otherCamera, (WorldList*)this->worldLst, 16, this->getWindowWidth() * .02f, this->getWindowHeight() * .02f);
            sink->setPosition(100, 0, 50);
            worldLst->push_back(sink);
        }
    }
    {
        //Create the infinite grass plane (the floor)
        WOCookingPhysx* wo = WOCookingPhysx::New(p,s,mCooking,grass, Vector(20, 5, 1));
        wo->setPosition(Vector(200, 0, 142));
        wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
        wo->upon_async_model_loaded([wo]()
            {
                wo->getModel()->getModelDataShared()->getModelMeshes().at(0)->getSkin().getMultiTextureSet().at(0) = *ManagerTex::loadTexAsync(ManagerEnvironmentConfiguration::getLMM()+"/models/Road.jpg"); // How "sharp" are the specular highlights (bigger is sharper, 1000 is very sharp, 10 is very dull)
            });
        wo->setLabel("Grass");
        worldLst->push_back(wo);
    }
    //import clouds 
    WO* cloud_1 = WO::New(ManagerEnvironmentConfiguration::getLMM() + "models/Clouds.fbx", Vector(0.6, 0.6, 0.6));
    WO* cloud_2 = WO::New(ManagerEnvironmentConfiguration::getLMM() + "models/Clouds.fbx", Vector(0.5, 0.5, 0.5));
    WO* cloud_3 = WO::New(ManagerEnvironmentConfiguration::getLMM() + "models/Clouds.fbx", Vector(0.6, 0.6, 0.6));
    WO* cloud_4 = WO::New(ManagerEnvironmentConfiguration::getLMM() + "models/Clouds.fbx", Vector(0.6, 0.6, 0.6));
    WO* cloud_5 = WO::New(ManagerEnvironmentConfiguration::getLMM() + "models/Clouds.fbx", Vector(0.5, 0.5, 0.5));

    WOImGui* gui = WOImGui::New(nullptr);
    
    drone = WO::New(wheeledCar, Vector(1, 1, 1));
    drone->setPosition(x, y, z);
    this->setActor(drone);
    worldLst->push_back(drone);

    
   gui->setLabel( "My Gui" );
   gui->subscribe_drawImGuiWidget(
       [this, gui, start_time, cloud_1, cloud_2, cloud_3, cloud_4, cloud_5]()
       {

           ImGuiStyle& st = ImGui::GetStyle();
           st.Colors[ImGuiCol_WindowBg] = ImVec4(0, 0, 0, 0.1);

           //Declaring thetas rotation angles
           float theta = 0.0f;
           float theta1 = 0.0f;
           float theta2 = 0.0f;
           ImGui::Begin("Ground Station");
           // Define an ImVec2-like struct
           
           struct MyVec2
           {
               float x, y;

               MyVec2(float _x, float _y)
               {
                   x = _x;
                   y = _y;
               }
           };

           // Load the image using OpenCV
           cv::Mat image = cv::imread(ManagerEnvironmentConfiguration::getLMM()+"images/frames.jpg");
           if(!image.empty()){
           // Convert the OpenCV Mat to a format that can be used with ImGui
           unsigned int textureID;
           glGenTextures(1, &textureID);
           glBindTexture(GL_TEXTURE_2D, textureID);
           glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image.cols, image.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, image.ptr());
           glGenerateMipmap(GL_TEXTURE_2D);

           float scale_factor = 0.2f; // adjust as needed
           ImGui::SetWindowSize(ImVec2(image.cols* scale_factor+35, image.rows* scale_factor+38));
           
           ImGui::Image((void*)(intptr_t)textureID, ImVec2(image.cols* scale_factor, image.rows* scale_factor));
           }
           ImGui::End();

           ImGui::Begin("Drone Controller");
           if (ImGui::Button("Start Engine")) {
               startEngine();
           }

           ImGui::Checkbox("Start Engine", &start);
           ImGui::Checkbox("Enable Gravity", &gravity);
           ImGui::SliderAngle("Global z", &theta, 90, -90);
           ImGui::SliderAngle("Global X", &theta1, 90, -90);
           ImGui::SliderAngle("Global Y", &theta2, 90, -90);

           ImGui::Checkbox("Forward", &up);
           ImGui::Checkbox("Move Back", &down);
           ImGui::Checkbox("Move Left", &left);
           ImGui::Checkbox("Move Right", &right);

           if (ImGui::Button("Shoot Now")) {
               shoot();
           }
           if (ImGui::Button("Cast Ray")) {
               castRay(this->drone);

           }
           if (ImGui::Button("Capture Image")) {
               captureImage();

               /*auto start_time = std::chrono::high_resolution_clock::now();
               while (true) {
                   auto current_time = std::chrono::high_resolution_clock::now();
                   auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
                   if (elapsed_time >= 3) {
                       break;
                   }
                   
               }*/
           }
           ImGui::Checkbox("Flip the capera", &flip);
           
           ImGui::Checkbox("Static Camera", &static_cam);
           auto end = std::chrono::high_resolution_clock::now();

           auto time_ms = std::chrono::duration_cast<std::chrono::seconds>(end - start_time).count();

           ImGui::Text("Time Elaspsed : %lld Seconds", time_ms);
           if (terrainCollision != NULL) {
               bool called_already = false;

               if ((this->drone->getPosition().z - terrainCollision.z) < 20.0) {
                   if (!engine->isCurrentlyPlaying("s")) {
                       alertSound(called_already);
                   }
                   called_already = true;
                   ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.0f, 0.0f, 1.0f));
                   terrainCollision.z;

               }
               else {
                   ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0f, 1.0f, 0.0f, 1.0f));
               }
               // Set text color to red
               ImGui::BulletText("Terrain is below at: %.2f", (this->drone->getPosition().z - terrainCollision.z));
               ImGui::PopStyleColor();
               
           }
           if (up == true) {

               auto t = std::chrono::high_resolution_clock::now();
               auto cam_look = this->getCamera()->getLookDirection();
               if (!static_cam) {
                   this->getCamera()->moveRelative(cam_look * this->getCamera()->getCameraVelocity());
                   //this->drone->setPosition(x, y, z);

                   if (cam->getLookDirection().x > 0) {
                       this->drone->setPosition(this->cam->getPosition().x + 15, this->cam->getPosition().y, this->cam->getPosition().z + 1);
                       sink->setPosition(this->drone->getPosition().x + 30, this->drone->getPosition().y + 20, this->drone->getPosition().z);
                   }
                   else {
                       this->drone->setPosition(this->cam->getPosition().x - 15, this->cam->getPosition().y, this->cam->getPosition().z + 1);
                       sink->setPosition(this->drone->getPosition().x - 30, this->drone->getPosition().y + 20, this->drone->getPosition().z);
                   }
               }
               else {
                   z = z + z / 5000;
                   this->cam->setPosition(0, 0, 155);
                   this->drone->setPosition(x, y, z);
                   sink->setPosition(100, 20, 160);
               }
               auto time_ms = std::chrono::duration_cast<std::chrono::seconds>(t - start_time).count();
               time = time_ms;
               ImGui::Text("Distance covered by UAV: %f", calculateSpeed(cam->getCameraVelocity(), time_ms));

           }
           if (gravity == true) {
               this->gravity = true;
           }
           else {
               this->gravity = false;
           }
           if (down) {
               auto cam_look = this->getCamera()->getLookDirection();
               if (!static_cam) {
                   this->getCamera()->moveRelative(-cam_look * this->getCamera()->getCameraVelocity());
                   //this->drone->setPosition(x, y, z);

                   if (cam->getLookDirection().x > 0) {
                       this->drone->setPosition(this->cam->getPosition().x + 15, this->cam->getPosition().y, this->cam->getPosition().z + 1);
                       sink->setPosition(this->drone->getPosition().x + 30, this->drone->getPosition().y + 20, this->drone->getPosition().z);
                   }
                   else {
                       this->drone->setPosition(this->cam->getPosition().x - 15, this->cam->getPosition().y, this->cam->getPosition().z + 1);
                       sink->setPosition(this->drone->getPosition().x - 30, this->drone->getPosition().y + 20, this->drone->getPosition().z);
                   }
               }
               else {
                   z = z - z / 5000;
                   this->cam->setPosition(0, 0, 155);
                   this->drone->setPosition(x, y, z);
                   sink->setPosition(100, 20, 160);
               }
           }
           if (left) {
               auto cam_look = this->getCamera()->getLookDirection();
               if (!static_cam) {
                   this->getCamera()->moveLeft();
                   //this->drone->setPosition(x, y, z);

                   if (cam->getLookDirection().x > 0) {
                       this->drone->setPosition(this->cam->getPosition().x + 15, this->cam->getPosition().y, this->cam->getPosition().z + 1);
                       sink->setPosition(this->drone->getPosition().x + 30, this->drone->getPosition().y + 20, this->drone->getPosition().z);
                   }
                   else {
                       this->drone->setPosition(this->cam->getPosition().x - 15, this->cam->getPosition().y, this->cam->getPosition().z + 1);
                       sink->setPosition(this->drone->getPosition().x - 30, this->drone->getPosition().y + 20, this->drone->getPosition().z);
                   }
               }
               else {
                   y = y + y / 100;
                   this->cam->setPosition(0, 0, 155);
                   this->drone->setPosition(x, y, z);
                   sink->setPosition(50, 20, 160);
                   sink->setPosition(100, 20, 160);
               }
           }
           if (right == true) {
               auto cam_look = this->getCamera()->getLookDirection();
               if (!static_cam) {
                   this->getCamera()->moveRelative(-cam_look * this->getCamera()->getCameraVelocity());
                   //this->drone->setPosition(x, y, z);

                   if (cam->getLookDirection().x > 0) {
                       this->drone->setPosition(this->cam->getPosition().x + 15, this->cam->getPosition().y, this->cam->getPosition().z + 1);
                       sink->setPosition(this->drone->getPosition().x + 30, this->drone->getPosition().y + 20, this->drone->getPosition().z);
                   }
                   else {
                       this->drone->setPosition(this->cam->getPosition().x - 15, this->cam->getPosition().y, this->cam->getPosition().z + 1);
                       sink->setPosition(this->drone->getPosition().x - 30, this->drone->getPosition().y + 20, this->drone->getPosition().z);
                   }
               }
               else {
                   y = y - y / 100;
                   this->cam->setPosition(0, 0, 155);
                   this->drone->setPosition(x, y, z);
                   sink->setPosition(100, 20, 160);
               }
           }
           drone->rotateAboutGlobalZ(theta / 5);
           drone->rotateAboutGlobalX(theta1 / 5);
           drone->rotateAboutGlobalY(theta2 / 5);

           cam->rotateAboutGlobalZ(theta / 20);
           cam->rotateAboutGlobalX(theta1 / 20);

           cam->rotateAboutGlobalY(theta2 / 20);
           if (ImGui::Button("Stop Engine")) {
               stopEngine();
           }
           ImGui::End();

           ImGui::Begin("Height-Speed Plots");
           ImPlot::SetNextAxesLimits(10, 100, 1000, 2000);
           if (ImPlot::BeginPlot("My Plot", "Cloud Speed", "Cloud Height")) {
               ImPlot::PlotStems("Stem Plot", x1.data(), y1.data(), x1.size());
               
               ImPlot::EndPlot();


           }
           ImGui::End();

           

           //Cloud Controller
           std::srand(std::time(nullptr)); // seed the random number generator
           int random_int = std::rand() % 201 + 1800;
           ImGui::Begin("Cloud Controller");
           ImGui::Checkbox("Dynamic Cloud", &dyna_cloud);
           ImGui::Checkbox("Move Opposite", &move_opposite);
           if (ImGui::Button("Cast Ray Up")) {
               castRayUp(this->drone, time_ms);

           }
            
           if (ImGui::Button("Cast Ray Angle Back")) {
               castRayAngleBack(this->drone);

           }

           if (ImGui::Button("Cast Ray Angle Front")) {
               castRayAngleFront(this->drone);

           }

           // Create a unique ID for the widget
           ImGuiID id = ImGui::GetID("Cloud Position");

           // Create a child window for the vector display
           ImGui::BeginChild(id, ImVec2(0, 100), true);

           // Display the vector values in a table
           int num_values = 3;
           int num_columns = 1;
           std::vector<std::string> axis;

           // Add some strings to the vector
           axis.push_back("X-Pos");
           axis.push_back("Y-Pos");
           axis.push_back("Z-Pos");
           int num_rows = (num_values + num_columns - 1) / num_columns;
           ImGui::Columns(num_columns, nullptr, false);
           ImGui::Text("Cloud Position:");
           for (size_t i = 0; i < num_values; ++i) {
               ImGui::Text("%s : %.2f",axis[i].c_str(), cloudCollision[i]);
               if ((i + 1) % num_rows != 0 && i != num_values - 1) {
                   ImGui::NextColumn();
               }
           }
           ImGui::Columns(1);

           // End the child window
           ImGui::EndChild();
           
           ImGui::BeginChild("Cloud Height");
           ImGui::Text("Cloud Height");
           ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 0.0f, 1.0f));
           if(cloudCollision.z>0){

           ImGui::BulletText("CLoud is above at: %.2f", (cloudCollision.z - (this->drone->getPosition().z)));
           }
           ImGui::PopStyleColor();
           //  };
           
           
           if(collisionPos.size()>2) {
           
               Vector end = collisionPos.back();
               Vector second_last = collisionPos.at(collisionPos.size() - 2);
              
               Vector result = Vector(end - second_last);
               
               
               float a = result.length();
               if(second_last!=Vector(0,0,0) && end!=Vector(0,0,0)){
                   ImGui::Text("Cloud Speed is: %.2f", a/timeT);
                   x1.push_back(a);
                   y1.push_back((cloudCollision.z - (this->drone->getPosition().z)));
                   if (end > second_last) {
                       ImGui::Text("Towards the UAV");
                   }
                   else {
                       ImGui::Text("Moving Backward from UAV");
                   }
               }

              
           
           }
           ImGui::EndChild();
          
           
          
         //  ImGui::BeginChild("Cloud Height");
         //  //float a = cloudCollision.z - (this->drone->getPosition().z);
         // // if (a > 0) {
         //  ImGui::Text("Cloud Height");
         //      ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.0f, 0.0f, 1.0f));
         //      ImGui::BulletText("CLoud is above at: %.2f", (cloudCollision.z - (this->drone->getPosition().z)));
         //      ImGui::PopStyleColor();
         ////  }
         //  ImGui::EndChild();

           ImGui::End();
           if (dyna_cloud) {
               cloud_x++;
           }
           if (move_opposite) {
               cloud_x--;
           }
           cloud_5->upon_async_model_loaded([cloud_5]() {

               cloud_5->getModel()->getModelDataShared()->getModelMeshes().at(0)->getSkin().getMultiTextureSet().at(0) = *ManagerTex::loadTexAsync(ManagerEnvironmentConfiguration::getLMM() + "models/clouds_color.jpg");
               cloud_5->getModel()->isUsingBlending(false);

               ModelMeshSkin& grassSkin = cloud_5->getModel()->getModelDataShared()->getModelMeshes().at(0)->getSkins().at(0);

               grassSkin.setAmbient(aftrColor4f(0.4f, 0.4f, 0.4f, 1.0f)); //Color of object when it is not in any light
               grassSkin.setDiffuse(aftrColor4f(1.0f, 1.0f, 1.0f, 1.0f)); //Diffuse color components (ie, matte shading color of this object)
               grassSkin.setSpecular(aftrColor4f(0.4f, 0.4f, 0.4f, 1.0f)); //Specular color component (ie, how "shiney" it is)
               grassSkin.setSpecularCoefficient(10);
               });
           cloud_5->setPosition(1000 * cloud_x / 5000, -1000, random_int);
           target5 = cloud_5;
           targets.push_back(cloud_5);
           cloud_4->upon_async_model_loaded([cloud_4]() {

               cloud_4->getModel()->getModelDataShared()->getModelMeshes().at(0)->getSkin().getMultiTextureSet().at(0) = *ManagerTex::loadTexAsync(ManagerEnvironmentConfiguration::getLMM() + "models/clouds_color.jpg");
               cloud_4->getModel()->isUsingBlending(false);

               ModelMeshSkin& grassSkin = cloud_4->getModel()->getModelDataShared()->getModelMeshes().at(0)->getSkins().at(0);

               grassSkin.setAmbient(aftrColor4f(0.4f, 0.4f, 0.4f, 1.0f)); //Color of object when it is not in any light
               grassSkin.setDiffuse(aftrColor4f(1.0f, 1.0f, 1.0f, 1.0f)); //Diffuse color components (ie, matte shading color of this object)
               grassSkin.setSpecular(aftrColor4f(0.4f, 0.4f, 0.4f, 1.0f)); //Specular color component (ie, how "shiney" it is)
               grassSkin.setSpecularCoefficient(10);
               });
           cloud_4->setPosition(0500 * cloud_x / 5000 + 100, 10, random_int);
           target4 = cloud_4;
           targets.push_back(cloud_4);
           cloud_2->upon_async_model_loaded([cloud_2]() {

               cloud_2->getModel()->getModelDataShared()->getModelMeshes().at(0)->getSkin().getMultiTextureSet().at(0) = *ManagerTex::loadTexAsync(ManagerEnvironmentConfiguration::getLMM() + "models/clouds_color.jpg");
               cloud_2->getModel()->isUsingBlending(false);

               ModelMeshSkin& grassSkin = cloud_2->getModel()->getModelDataShared()->getModelMeshes().at(0)->getSkins().at(0);

               grassSkin.setAmbient(aftrColor4f(0.4f, 0.4f, 0.4f, 1.0f)); //Color of object when it is not in any light
               grassSkin.setDiffuse(aftrColor4f(1.0f, 1.0f, 1.0f, 1.0f)); //Diffuse color components (ie, matte shading color of this object)
               grassSkin.setSpecular(aftrColor4f(0.4f, 0.4f, 0.4f, 1.0f)); //Specular color component (ie, how "shiney" it is)
               grassSkin.setSpecularCoefficient(10);
               });
           cloud_2->setPosition(050 * cloud_x / 5000 + 100, -500, random_int);
           target2 = cloud_2;
           targets.push_back(cloud_2);
           cloud_1->upon_async_model_loaded([cloud_1]() {

               cloud_1->getModel()->getModelDataShared()->getModelMeshes().at(0)->getSkin().getMultiTextureSet().at(0) = *ManagerTex::loadTexAsync(ManagerEnvironmentConfiguration::getLMM() + "models/clouds_color.jpg");
               cloud_1->getModel()->isUsingBlending(false);

               ModelMeshSkin& grassSkin = cloud_1->getModel()->getModelDataShared()->getModelMeshes().at(0)->getSkins().at(0);

               grassSkin.setAmbient(aftrColor4f(0.4f, 0.4f, 0.4f, 1.0f)); //Color of object when it is not in any light
               grassSkin.setDiffuse(aftrColor4f(1.0f, 1.0f, 1.0f, 1.0f)); //Diffuse color components (ie, matte shading color of this object)
               grassSkin.setSpecular(aftrColor4f(0.4f, 0.4f, 0.4f, 1.0f)); //Specular color component (ie, how "shiney" it is)
               grassSkin.setSpecularCoefficient(10);
               });
           cloud_1->setPosition(-150 * cloud_x / 5000, 500, random_int);
           target1 = cloud_1;
           targets.push_back(cloud_1);
           cloud_3->upon_async_model_loaded([cloud_3]() {

               cloud_3->getModel()->getModelDataShared()->getModelMeshes().at(0)->getSkin().getMultiTextureSet().at(0) = *ManagerTex::loadTexAsync(ManagerEnvironmentConfiguration::getLMM() + "models/clouds_color.jpg");
               cloud_3->getModel()->isUsingBlending(false);

               ModelMeshSkin& grassSkin = cloud_3->getModel()->getModelDataShared()->getModelMeshes().at(0)->getSkins().at(0);

               grassSkin.setAmbient(aftrColor4f(0.4f, 0.4f, 0.4f, 1.0f)); //Color of object when it is not in any light
               grassSkin.setDiffuse(aftrColor4f(1.0f, 1.0f, 1.0f, 1.0f)); //Diffuse color components (ie, matte shading color of this object)
               grassSkin.setSpecular(aftrColor4f(0.4f, 0.4f, 0.4f, 1.0f)); //Specular color component (ie, how "shiney" it is)
               grassSkin.setSpecularCoefficient(10);
               });
           cloud_3->setPosition(50 * cloud_x / 5000, -100, random_int);
           target3 = cloud_3;
           targets.push_back(cloud_3);
      } );
   this->worldLst->push_back( gui );
   worldLst->push_back(cloud_2);
   worldLst->push_back(cloud_1);
   worldLst->push_back(cloud_3);
   worldLst->push_back(cloud_4);
   worldLst->push_back(cloud_5);

   createFinalProjectCodeWayPoints();

   {
       std::string elevationPath(ManagerEnvironmentConfiguration::getLMM() + "models/nc.tif");
       std::string texturePath(ManagerEnvironmentConfiguration::getLMM() + "models/nc.bmp");

       // woodland
       float top = 34.2072593790098f;
       float bottom = 33.9980272592999f;

       float left = -118.65234375f;
       float right = -118.443603515625f;

       float vert = top - bottom;
       float horz = right - left;


       VectorD offset((top + bottom) / 2, (left + right) / 2, 0);


       auto centerOfWorld = offset.toVecS().toECEFfromWGS84();
       gravityDirection = -centerOfWorld;
       gravityDirection.normalize();

       VectorD scale = VectorD(0.1f, 0.1f, 0.1f);
       VectorD upperLeft(top, left, 0);
       VectorD lowerRight(bottom, right, 0);


       grid = WOTerrainPhysx::New(p, s, mCooking, upperLeft, lowerRight, offset, scale, elevationPath, texturePath);
       grid->setPosition(0.0f, 0.0f,0.0f);

       grid->setLabel("grid");
       worldLst->push_back((WO*)grid);
       this->target = this->grid;
   }
  
}


void GLViewFinalProjectCode::createFinalProjectCodeWayPoints()
{
   // Create a waypoint with a radius of 3, a frequency of 5 seconds, activated by GLView's camera, and is visible.
   WayPointParametersBase params(this);
   params.frequency = 5000;
   params.useCamera = true;
   params.visible = true;
   wayPt = WOWayPointSpherical::New( params, 3 );
   wayPt->setPosition(cam->getLookDirection().x * 20, cam->getLookDirection().y * 20, cam->getLookDirection().z * 20 );

   Vector Pos;
   wayPt->getNearestPointWhereLineIntersectsMe(Vector(50, 0, 3), Vector(0, 0, 3), Pos, true, true, NULL, NULL);
   worldLst->push_back( wayPt );
}

void GLViewFinalProjectCode::startEngine() {
    engine->play3D("tt", vec3df(0.0,0.0,0.0), true);
}
bool soundPlayed = false;
void GLViewFinalProjectCode::alertSound(bool flag) {
  
    if(!flag){
    engine->play3D("s", vec3df(0.0,0.0,0.0));
    engine->setSoundVolume(0.2);
    soundPlayed = !flag;
    //terrainCollision = NULL;
    }
    }
void GLViewFinalProjectCode::stopEngine() {
    engine->stopAllSounds();
}   

void GLViewFinalProjectCode::castRay(WO* v) {
    
    ray = WORay::New(Vector(v->getPosition().x, v->getPosition().y, v->getPosition().z), Vector(v->getPosition().x, v->getPosition().y, -100));
    ray->isVisible = false;
    worldLst->push_back(this->ray);
    Vector head, tail, contactPt;
    this->ray->getRayHeadAndTail(head, tail);
    {
        AftrGeometricTerm hit = AftrGeometricTerm::geoUNDEFINED;
        {
            boost::timer::auto_cpu_timer t(std::cout, "Boost Wall Time %w\n");
            hit = this->target->getNearestPointWhereLineIntersectsMe(tail, head, contactPt, true, false);
            std::cout << "Contact Point ho yo chai hai ta" << contactPt << std::endl;
        }
        if (hit == AftrGeometricTerm::geoSUCCESS)
        {



        }
        if ((contactPt.z) < 100) {

        }
        
        terrainCollision = contactPt;
        
        
    }

}

void GLViewFinalProjectCode::castRayUp(WO* u,auto time) {
        

        ray = WORay::New(Vector(u->getPosition().x, u->getPosition().y, u->getPosition().z), Vector(u->getPosition().x, u->getPosition().y, 2100));
        ray->isVisible = false;
        worldLst->push_back(this->ray);
    
    Vector head, tail, contactPt;
    this->ray->getRayHeadAndTail(head, tail);
    {
        AftrGeometricTerm hit = AftrGeometricTerm::geoUNDEFINED;
        {

            for (int i = 0; i < targets.size(); i++) {
                hit = this->targets[i]->getNearestPointWhereLineIntersectsMe(tail, head, contactPt, true, false);

                if (hit == AftrGeometricTerm::geoSUCCESS)
                {
                    
                    
                    
                   
                    ray->isVisible = true;
                    if (std::find(hittable.begin(), hittable.end(), targets[i]) == hittable.end()) {
                        hittable.push_back(targets[i]);
                        ray1 = WORay::New(contactPt, this->lightPos);
                        ray1->isVisible = true;
                        worldLst->push_back(ray1);

                        ray2 = WORay::New(targets[i]->getPosition(), contactPt + 0.5);
                        ray2->isVisible = true;
                        worldLst->push_back(ray2);

                        Vector diff1 = (this->lightPos - contactPt);
                        std::cout << diff1 << std::endl;

                        Vector diff2 = ((contactPt + 0.5) - targets[i]->getPosition());
                        std::cout << diff2 << std::endl;
                        float mag1 = diff1.magnitude();
                        float mag2 = diff2.length();
                        std::cout << "Mag 1" << mag1 << std::endl;
                        Vector normal1 = diff1 / mag1;
                        Vector normal2 = diff2 / mag2;

                        if (normal1.dotProduct(normal2) <0) {
                          
                            ModelMeshSkin& grassSkin = targets[i]->getModel()->getModelDataShared()->getModelMeshes().at(0)->getSkins().at(0);

                            grassSkin.setAmbient(aftrColor4f(0.0f, 0.0f, 0.0f, 1.0f)); //Color of object when it is not in any light
                            grassSkin.setDiffuse(aftrColor4f(1.0f, 1.0f, 1.0f, 1.0f)); //Diffuse color components (ie, matte shading color of this object)
                            grassSkin.setSpecular(aftrColor4f(0.4f, 0.4f, 0.4f, 1.0f)); //Specular color component (ie, how "shiney" it is)
                            grassSkin.setSpecularCoefficient(10);
                        }
                       
                    }
                }
            }


        }
        cloudCollision = contactPt;
        auto end = std::chrono::high_resolution_clock::now();
        
        times.push_back(time);
        collisionPos.push_back(cloudCollision);
    }

}

void GLViewFinalProjectCode::castRayAngleBack(WO* u) {
    
        ray = WORay::New(Vector(u->getPosition().x, u->getPosition().y, u->getPosition().z), Vector(u->getPosition().x -500, u->getPosition().y, 2500));
        ray->isVisible = true;
        worldLst->push_back(this->ray);
    
    Vector head, tail, contactPt;
    this->ray->getRayHeadAndTail(head, tail);
    {
        AftrGeometricTerm hit1 = AftrGeometricTerm::geoUNDEFINED;
        {
            boost::timer::auto_cpu_timer t(std::cout, "Boost Wall Time %w\n");
            hit1 = this->target1->getNearestPointWhereLineIntersectsMe(tail, head, contactPt, true, false);
            std::cout << "Contact Point ho yo chai hai ta" << contactPt << std::endl;

        }
        AftrGeometricTerm hit2 = AftrGeometricTerm::geoUNDEFINED;
        {
            boost::timer::auto_cpu_timer t(std::cout, "Boost Wall Time %w\n");
            hit2 = this->target2->getNearestPointWhereLineIntersectsMe(tail, head, contactPt, true, false);
            std::cout << "Contact Point ho yo chai hai ta" << contactPt << std::endl;
        }
        AftrGeometricTerm hit3 = AftrGeometricTerm::geoUNDEFINED;
        {
            boost::timer::auto_cpu_timer t(std::cout, "Boost Wall Time %w\n");
            hit3 = this->target3->getNearestPointWhereLineIntersectsMe(tail, head, contactPt, true, false);
            std::cout << "Contact Point ho yo chai hai ta" << contactPt << std::endl;
        }
        AftrGeometricTerm hit4 = AftrGeometricTerm::geoUNDEFINED;
        {
            boost::timer::auto_cpu_timer t(std::cout, "Boost Wall Time %w\n");
            hit4 = this->target4->getNearestPointWhereLineIntersectsMe(tail, head, contactPt, true, false);
            std::cout << "Contact Point ho yo chai hai ta" << contactPt << std::endl;
        }
        AftrGeometricTerm hit5 = AftrGeometricTerm::geoUNDEFINED;
        {
            boost::timer::auto_cpu_timer t(std::cout, "Boost Wall Time %w\n");
            hit5 = this->target5->getNearestPointWhereLineIntersectsMe(tail, head, contactPt, true, false);
            std::cout << "Contact Point ho yo chai hai ta" << contactPt << std::endl;
        }
        if (hit5 == AftrGeometricTerm::geoSUCCESS)
        {



        }
        if ((contactPt.z) < 100) {

        }

        cloudCollision = contactPt;


    }

}


void GLViewFinalProjectCode::castRayAngleFront(WO* u) {
    
        ray = WORay::New(Vector(u->getPosition().x, u->getPosition().y, u->getPosition().z), Vector(u->getPosition().x + 500, u->getPosition().y, 2500));
        ray->isVisible = true;
        worldLst->push_back(this->ray);
    
    Vector head, tail, contactPt;
    this->ray->getRayHeadAndTail(head, tail);
    {
        AftrGeometricTerm hit1 = AftrGeometricTerm::geoUNDEFINED;
        {
            boost::timer::auto_cpu_timer t(std::cout, "Boost Wall Time %w\n");
            hit1 = this->target1->getNearestPointWhereLineIntersectsMe(tail, head, contactPt, true, false);
            std::cout << "Contact Point ho yo chai hai ta" << contactPt << std::endl;

        }
        AftrGeometricTerm hit2 = AftrGeometricTerm::geoUNDEFINED;
        {
            boost::timer::auto_cpu_timer t(std::cout, "Boost Wall Time %w\n");
            hit2 = this->target2->getNearestPointWhereLineIntersectsMe(tail, head, contactPt, true, false);
            std::cout << "Contact Point ho yo chai hai ta" << contactPt << std::endl;
        }
        AftrGeometricTerm hit3 = AftrGeometricTerm::geoUNDEFINED;
        {
            boost::timer::auto_cpu_timer t(std::cout, "Boost Wall Time %w\n");
            hit3 = this->target3->getNearestPointWhereLineIntersectsMe(tail, head, contactPt, true, false);
            std::cout << "Contact Point ho yo chai hai ta" << contactPt << std::endl;
        }
        AftrGeometricTerm hit4 = AftrGeometricTerm::geoUNDEFINED;
        {
            boost::timer::auto_cpu_timer t(std::cout, "Boost Wall Time %w\n");
            hit4 = this->target4->getNearestPointWhereLineIntersectsMe(tail, head, contactPt, true, false);
            std::cout << "Contact Point ho yo chai hai ta" << contactPt << std::endl;
        }
        AftrGeometricTerm hit5 = AftrGeometricTerm::geoUNDEFINED;
        {
            boost::timer::auto_cpu_timer t(std::cout, "Boost Wall Time %w\n");
            hit5 = this->target5->getNearestPointWhereLineIntersectsMe(tail, head, contactPt, true, false);
            std::cout << "Contact Point ho yo chai hai ta" << contactPt << std::endl;
        }
        if (hit5 == AftrGeometricTerm::geoSUCCESS)
        {



        }
        if ((contactPt.z) < 100) {

        }

        cloudCollision = contactPt;


    }

}


void GLViewFinalProjectCode::shoot() {
    std::string path(ManagerEnvironmentConfiguration::getSMM() + "/models/monkeyFace.wrl");
   
    Vector cam_pos = this->cam->getPosition();
    WOPhysxBox* wo = WOPhysxBox::New(p, s, path, Vector(2, 2, 2), drone->getPosition().x, drone->getPosition().y, drone->getPosition().z);
    Vector cam_look = this->cam->getLookDirection().normalizeMe();
    wo->addForces(cam_look.x, cam_look.y, cam_look.z);
    wo->updatePoseFromPhysx();
    worldLst->push_back(wo);
    engine->play3D("t", vec3df(0.0,0.0,0.0));
}

float GLViewFinalProjectCode::calculateSpeed(float v,auto start) {
    
    /*auto end = std::chrono::high_resolution_clock::now();
    auto time_ms = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();*/

     
    return v*start;
}

float GLViewFinalProjectCode::Getdist(Vector point) {
    Vector s = Vector(5, 0, 0);

    float rad = 2;
    float sphereDist = ((point - s).length()) - rad;
    float planeDist = point.y;
    //float d = (sphereDist < planeDist) ? sphereDist : planeDist;
    float d = sphereDist;
    return d;
}

float GLViewFinalProjectCode::rayMarcher(Vector point, Vector direction) {

    int Max_steps = 100;
    float Max_dist = 100.0f;
    float surf_dist = 0.01;

    float d0 = 0.0f;
    for (int i = 0; i < Max_steps; i++) {
        Vector p = point + d0 * direction;
        float ds = this->Getdist(p);
        d0 += ds;
        if (d0 > Max_dist || ds < surf_dist) {
            break;
        }
    }
    return d0;
}

void GLViewFinalProjectCode::captureImage() {
    std::vector<GLubyte>buffer(this->getWindowHeight() * this->getWindowWidth() * 3);

    //Read pixels from frame buffer
    glReadPixels(0, 0, this->otherCamera->getCameraViewport().getWidth(), this->otherCamera->getCameraViewport().getHeight(), GL_RGB, GL_UNSIGNED_BYTE, &buffer[0]);

    //Flip the Image Vertically
    cv::Mat image(this->otherCamera->getCameraViewport().getHeight(), this->otherCamera->getCameraViewport().getWidth(), CV_8UC3, &buffer[0]);
   cv::flip(image, image, 0);
    cv::imwrite(ManagerEnvironmentConfiguration::getLMM() + "images/frames.jpg", image);
    
}
void load_texture(const char* filename, GLuint* out_texture)
{
    int width, height, channels;
    stbi_set_flip_vertically_on_load(true);
    unsigned char* data = stbi_load(filename, &width, &height, &channels, 0);
    if (data)
    {
        glGenTextures(1, out_texture);
        glBindTexture(GL_TEXTURE_2D, *out_texture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);
        stbi_image_free(data);
    }
}

float GLViewFinalProjectCode::calculateCloudSpeed(std::vector<Vector>collisionPos, std::vector<int> time) {

    
        Vector end = Vector(collisionPos.back());
        Vector second_last = Vector(collisionPos[collisionPos.size() - 1]);
        std::cout << end << std::endl;
        Vector result = Vector(end - second_last);
        std::cout << "Length of distances" << result;
        return result.length();
        
}

/*

void Aftr::GLViewNewModule::loadMap()
{

   this->worldLst = new WorldList(); //WorldList is a 'smart' vector that is used to store WO*'s
   this->actorLst = new WorldList();
   this->netLst = new WorldList();

   ManagerOpenGLState::GL_CLIPPING_PLANE = 1000.0;
   ManagerOpenGLState::GL_NEAR_PLANE = 0.1f;
   ManagerOpenGLState::enableFrustumCulling = false;
   Axes::isVisible = true;
   this->glRenderer->isUsingShadowMapping( false ); //set to TRUE to enable shadow mapping, must be using GL 3.2+

   this->cam->setPosition( 15,15,10 );

   std::string shinyRedPlasticCube( ManagerEnvironmentConfiguration::getSMM() + "/models/cube4x4x4redShinyPlastic_pp.wrl" );
   std::string wheeledCar( ManagerEnvironmentConfiguration::getSMM() + "/models/rcx_treads.wrl" );
   std::string grass( ManagerEnvironmentConfiguration::getSMM() + "/models/grassFloor400x400_pp.wrl" );
   std::string human( ManagerEnvironmentConfiguration::getSMM() + "/models/human_chest.wrl" );


   {

      float ga = 0.1f; //Global Ambient Light level for this module
      ManagerLight::setGlobalAmbientLight( aftrColor4f( ga, ga, ga, 1.0f ) );
      WOLight* light = WOLight::New();
      light->isDirectionalLight( true );
      light->setPosition( Vector( 100, 0, 10 ) );

      light->getModel()->setDisplayMatrix( Mat4::rotateIdentityMat( { 0, 1, 0 }, 90.0f * Aftr::DEGtoRAD ) );
      light->setLabel( "Light" );
      light_normal = light->getNormalDirection();
      this->targetLight = light;
      lightPos = light->getPosition();
      worldLst->push_back( light );

   }
   {
       moon = WO::New(ManagerEnvironmentConfiguration::getSMM()+"models/sphereR2p5Moon.wrl", Vector(1, 1, 1));
       moon->setPosition(20, 010, 10);
       target.push_back(moon);
       worldLst->push_back(moon);
   }
   {
       WO* wo = WO::New(shinyRedPlasticCube, Vector(1, 1, 1));
       wo->setPosition(10, 5, 0);
       target.push_back(wo);
       worldLst->push_back(wo);
   }
   {
       WO* wo = WO::New(shinyRedPlasticCube, Vector(1, 1, 1));
       wo->setPosition(-5, 5, 0);
       target.push_back(wo);
       worldLst->push_back(wo);
   }
   {
       WO* wo = WO::New(shinyRedPlasticCube, Vector(1, 1, 1));
       wo->setPosition(-10, 10, 2);
       target.push_back(wo);
       worldLst->push_back(wo);
   }
   {
       WO* wo = WO::New(shinyRedPlasticCube, Vector(1, 1, 1));
       wo->setPosition(10, 0, 0);
       target.push_back(wo);
       worldLst->push_back(wo);
   }

   WOImGui* gui = WOImGui::New( nullptr );
   gui->setLabel( "My Gui" );
   gui->subscribe_drawImGuiWidget(
      [this, gui]()
      {
           ImGui::Begin("Testing");
           if (ImGui::Button("Cast the ray")) {
               castRay();
           }
           if (ImGui::Button("March Ray")) {
               rayMarch();
           }
           ImGui::Text("Signed Distance function:: %.2f", signed_distance);
           ImGui::End();
      } );
   this->worldLst->push_back( gui );

}




void GLViewNewModule::castRay() {

    Vector v = cam->getPosition();
    // ray = WORay::New(cam->getPosition(), Vector(cam->getLookDirection().x * 5, cam->getLookDirection().y * 5, cam->getLookDirection().y*5));
    // ray = WORay::New(Vector(0,0, 0), Vector(100,0, 2));
    const auto aspect_ratio = 16.0 / 9.0;
    const int image_width = 20;
    const int image_height = static_cast<int>(image_width / aspect_ratio);


    auto origin = Vector(0, 0, 0);
    auto horizontal = (static_cast<double>(this->cam->getCameraViewport().getWidth()), 0, 0);
    auto vertical = (0, this->cam->getCameraViewport().getHeight(), 0);
    auto lower = Vector(cam->getCameraViewport().getX(), cam->getCameraViewport().getY());

    for (int j = image_height; j >= 0; --j) {

        for (int i = 0; i < image_width; ++i) {

            auto u = double(i) / (image_width - 1);
            auto v = double(j) / (image_height - 1);
            for (int z = 0; z < 100; z++) {
                ray = WORay::New(origin, Vector(u * this->cam->getCameraViewport().getWidth(), 0, 0) + Vector(0, v * this->cam->getCameraViewport().getHeight(), z));
            }
            ray->isVisible = false;
            worldLst->push_back(this->ray);


            Vector head, tail, contactPt;
            this->ray->getRayHeadAndTail(head, tail);
            {
                AftrGeometricTerm hit = AftrGeometricTerm::geoUNDEFINED;
                {

                    for (int i = 0; i < target.size(); i++) {
                        hit = this->target[i]->getNearestPointWhereLineIntersectsMe(tail, head, contactPt, true, false);

                        if (hit == AftrGeometricTerm::geoSUCCESS)
                        {
                            ray->isVisible = true;
                            if (std::find(hittable.begin(), hittable.end(), target[i]) == hittable.end()) {
                                hittable.push_back(target[i]);
                                ray1 = WORay::New(contactPt, this->lightPos);
                                ray1->isVisible = true;
                                worldLst->push_back(ray1);

                                ray2 = WORay::New(target[i]->getPosition(), contactPt+0.5);
                                ray2->isVisible = true;
                                worldLst->push_back(ray2);

                                Vector diff1 = (this->lightPos - contactPt);
                                std::cout << diff1 << std::endl;

                                Vector diff2 = ((contactPt + 0.5) - target[i]->getPosition());
                                std::cout << diff2 << std::endl;
                                float mag1 = diff1.magnitude();
                                float mag2 = diff2.length();
                                std::cout << "Mag 1" << mag1<<std::endl;
                                Vector normal1 = diff1 / mag1;
                                Vector normal2 = diff2 / mag2;

                                if (normal1.dotProduct(normal2) < 0) {

                                    ModelMeshSkin& grassSkin = target[i]->getModel()->getModelDataShared()->getModelMeshes().at(0)->getSkins().at(0);

                                    grassSkin.setAmbient(aftrColor4f(0.0f, 0.0f, 0.0f, 1.0f)); //Color of object when it is not in any light
                                    grassSkin.setDiffuse(aftrColor4f(1.0f, 1.0f, 1.0f, 1.0f)); //Diffuse color components (ie, matte shading color of this object)
                                    grassSkin.setSpecular(aftrColor4f(0.4f, 0.4f, 0.4f, 1.0f)); //Specular color component (ie, how "shiney" it is)
                                    grassSkin.setSpecularCoefficient(10);
                                }
                            }
                        }
                    }


                }
            }

        }
    }


}


float GLViewNewModule::Getdist(WORay* ray,Vector p) {

    Vector head, tail, contactPt;
    this->ray->getRayHeadAndTail(head, tail);
    {
        AftrGeometricTerm hit = AftrGeometricTerm::geoUNDEFINED;
        {

            for (int i = 0; i < target.size(); i++) {
                hit = this->target[i]->getNearestPointWhereLineIntersectsMe(tail, head, contactPt, true, false);

                if (hit == AftrGeometricTerm::geoSUCCESS)
                {
                    Vector diff = contactPt - p;
                    float magnitude = diff.length();
                    signed_distance = magnitude - 1;
                    marchPoint = contactPt;
                }
            }
        }
    }

    return signed_distance;
}

float GLViewNewModule::rayMarcher(Vector point, Vector direction) {

    int Max_steps = 100;
    float Max_dist = 100.0f;
    float surf_dist = 0.01;
    float d0 = 10.0f;

    for (int i = 0; i < Max_steps; i++) {
        Vector p = point + d0 * direction;
        ray = WORay::New(point, d0 * direction);
        ray->isVisible = true;
        worldLst->push_back(ray);
        float ds = this->Getdist(ray, point);
        d0 += ds;
        if (marchPoint != Vector(0,0,0)) {
            std::cout << "Hami yaha chau !!!" << std::endl;
            point = marchPoint;
        }
        std::cout << "Marched Distance!!!!!" << d0 << std::endl;
        if (d0 > Max_dist || ds < surf_dist) {
            break;
        }
    }

    return d0;
}

void GLViewNewModule::rayMarch() {

    const auto aspect_ratio = 16.0 / 9.0;
    const int image_width = 20;
    const int image_height = static_cast<int>(image_width / aspect_ratio);

    auto origin = Vector(0, 0, 0);
    auto horizontal = (static_cast<double>(this->cam->getCameraViewport().getWidth()), 0, 0);
    auto vertical = (0, this->cam->getCameraViewport().getHeight(), 0);
    auto lower = Vector(cam->getCameraViewport().getX(), cam->getCameraViewport().getY());

    int num_rays = 20;
    double radius = 100;

    for (int i = 0; i < num_rays; i++) {
        double azimuth = 2 * M_PI * i / num_rays;
        for (int j = 0; j < num_rays; j++) {
            double elevation = M_PI * j / num_rays;
            double x = radius * sin(elevation) * cos(azimuth);
            double y = radius * sin(elevation) * sin(azimuth);
            double z = radius * cos(elevation);


            rayMarcher(marchPoint, Vector(x, y, z).normalizeMe());
        }
    }
}

*/