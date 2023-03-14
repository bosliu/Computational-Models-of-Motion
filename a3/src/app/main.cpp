#include "application.h"
#include <imgui.h>

#include <iostream>
#include <math.h>
#include <deque>
#include <chrono>

#include "../boids/boids.h"

#define dim 2
#define T float

using VectorXT = Matrix<T, Eigen::Dynamic, 1>;
using TVStack = Matrix<T, dim, Eigen::Dynamic>;
using TV = Vector<T, dim>;


class TestApp : public Application
{
#define COLOR_OUT    nvgRGBA(220,50,50,255)
#define COLOR_IN     nvgRGBA(50,50,220,255)
#define COLOR_SOLVED nvgRGBA(50,220,50,255)




public:

    TestApp(int w, int h, const char * title) : Application(title, w, h) {
        
        ImGui::StyleColorsClassic();
        
        const char* name = IMGUI_FONT_FOLDER"/Cousine-Regular.ttf";
        nvgCreateFont(vg, "sans", name);
        
    }

    void process() override {
        std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
        if(std::chrono::duration_cast<std::chrono::microseconds>(now-lastFrame).count() >= 10./60. * 1.e6)
        {
            if(keyDown[GLFW_KEY_R])
            {
                boids = Boids<T, dim>(40);
                boids.initializePositions(initialVel);
            }
            if(keyDown[GLFW_KEY_SPACE])
                boids.pause();
            if(keyDown[GLFW_KEY_ESCAPE])
                exit(0);
            lastFrame = now;
        }
    }

    void drawImGui() override {

        using namespace ImGui;

       const char* initVel[] = {"Zero", "Random"};
       const char* update_names[] = {"ExplicitEuler","SymplecticEuler", "ExplicitMidpoint"};
       const char* names[] = {"FreeFall", "cohesion", "Alignment", "separation", "collisionAvoidance","leading","CollaborationAndAdversary"};
       
       Begin("Menu");
       
       Combo("Initial Velocities", (int*)&initialVel, initVel, 2);
       if(initialVel != initialVelTemp)
       {
           boids.initializePositions(initialVel);
           initialVelTemp = initialVel;

       }
        float fsmin = 0.5f;
        float fsmax = 5.0f;
        SliderScalar("Force Scaler", ImGuiDataType_Float, &f_scaler_read, &fsmin, &fsmax);
        float hmin = 0.001f;
        float hmax = 0.5f;
        SliderScalar("Step Size", ImGuiDataType_Float, &h_read, &hmin, &hmax);
        
       Combo("Boids Behavior", (int*)&currentMethod, names, 7);
       if(currentMethod == FREEFALL)
       {
           drawObtacle = false;
       }
       if(currentMethod == SEPARATION || currentMethod == ALIGNMENT || currentMethod == COHESION )
       {
           float rmin  = 0.01f;
           float rmax = 2.0f;
           
           SliderScalar("Range", ImGuiDataType_Float, &range_read, &rmin, &rmax);
           drawObtacle = false;
       }


       if(currentMethod == LEADING)
       {
           float rmin = 0.01f;
           float rmax = 2.0f;
           SliderScalar("Range", ImGuiDataType_Float, &range_read, &rmin, &rmax); SameLine();
           
           
           Checkbox("Obstacle", &drawObtacle);
           
       }
           
       
       else  cursorFind = false;

       if (currentMethod == COLLISION_AVOIDANCE)
       {
           
           
            drawObtacle = true;
       }

       Combo("update Method", (int*)&updateRule, update_names, 3);


       End();
    }

    void drawNanoVG() override {
        
        
        boids.read_parameters(h_read, f_scaler_read, range_read, obstacle_read, drawObtacle);
        boids.updateBehavior(currentMethod, updateRule);
        TVStack boids_pos = boids.getPositions();

        auto shift_01_to_screen = [](TV pos_01, T scale, T width, T height)
        {
            return TV(0.5 * (0.5 - scale) * width + scale * pos_01[0] * width, 0.5 * (0.5 - scale) * height + scale * pos_01[1] * height);
        };
        auto shift_screen_to_01 = [](TV screen_pos, T scale, T width, T height)
        {
            return TV((screen_pos[0] - 0.5 * (0.5-scale)* width)/(scale * width),(screen_pos[1]-0.5 * (0.5 - scale) * height) / (scale * height));
        };
        if (drawObtacle)
        {
            T scale = 0.3f;
            circle_obstacle_start = shift_01_to_screen(obstacle_read.pos, scale, width, height);
            circle = {circle_obstacle_start, scale * obstacle_read.radius * width, COLOR_SOLVED, nvgRGBA(10, 10, 10, 255)};
            auto drawCircle = [this](const Circle &circle)
            {
                nvgBeginPath(vg);
                nvgCircle(vg, circle.pos[0], circle.pos[1], circle.radius);
                nvgFillColor(vg, circle.colorFill);
                nvgFill(vg);
                nvgStrokeColor(vg, circle.colorStroke);
                nvgStrokeWidth(vg, 3.0f);
                nvgStroke(vg);
            };
            drawCircle(circle);
        }
       
       
       
       
       
       
       
        for(int i = 0; i < boids.getParticleNumber(); i++)
        {
            TV pos = boids_pos.col(i);
            nvgBeginPath(vg);
    
            // just map position from 01 simulation space to scree space
            // feel free to make changes
            // the only thing that matters is you have pos computed correctly from your simulation
            T scale = 0.3f;
            TV screen_pos = shift_01_to_screen(TV(pos[0], pos[1]), scale, width, height);
            nvgCircle(vg, screen_pos[0], screen_pos[1], 2.f);
            if(i ==0 && cursorFind)
            {
                nvgFillColor(vg, COLOR_IN);

            }
            else nvgFillColor(vg, COLOR_OUT);
            
            
            nvgFill(vg);

        }
       

        cursorFind=true;
        if (cursorFind)
        {
           if (mouseState.lButtonPressed && mouseState.lastMouseX < width)
           {
           cursorPosDown =TV(mouseState.lastMouseX, mouseState.lastMouseY);
           }
           T scale = 0.3f;
           TV target_pos_read = shift_screen_to_01(cursorPosDown, scale, width, height);
           boids.getCursor(target_pos_read);

        }
    }


protected:
    void mouseButtonPressed(int button, int mods) override {

    }

    void mouseButtonReleased(int button, int mods) override {
        
    }

private:
    int loadFonts(NVGcontext* vg)
    {
        int font;
        font = nvgCreateFont(vg, "sans", "../example/Roboto-Regular.ttf");
        if (font == -1) {
            printf("Could not add font regular.\n");
            return -1;
        }
        font = nvgCreateFont(vg, "sans-bold", "../example/Roboto-Bold.ttf");
        if (font == -1) {
            printf("Could not add font bold.\n");
            return -1;
        }
        return 0;
    }

private:

    MethodTypes currentMethod = FREEFALL;

    TimeIntegrationSchemes updateRule = EXPLICITEULER;
    InitTypes initialVel = ZERO;
    InitTypes initialVelTemp = ZERO;
    Boids<T, dim> boids = Boids<T, dim>(40);
    float h_read = 0.03f;
    float f_scaler_read = 2.0f;
    float range_read = 0.5f;
    bool drawObtacle = false;
    TV circle_obstacle_start;
    std::chrono::high_resolution_clock::time_point lastFrame;

    struct Circle
    {
        TV pos=TV(0.5f, 0.5f);
        float radius =0.2f;
        NVGcolor colorFill, colorStroke;
    }circle;

    Boids<T, dim>::Obstacle obstacle_read = {TV(0.5f, 0.5f), 0.2f};

    bool cursorFind = false;
    TV cursorPosDown = TV(360.0f, 360.0f);
};

int main(int, char**)
{
    int width = 720;
    int height = 720;
    
    TestApp app(width, height, "Assignment 3 Boids");
    app.run();

    return 0;
}
