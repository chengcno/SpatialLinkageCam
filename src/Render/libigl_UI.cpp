///////////////////////////////////////////////////////////////
//
// libigl_UI.h
//
//   User interface with libigl and imGUI
//
// by Yucheng Sun and Yingjie Cheng
//
// 08/Aug/2022
//
//
///////////////////////////////////////////////////////////////

#include "libigl_UI.h"

#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include "Utility/HelpTypedef.h"
#include "Render/libigl_Render.h"

using std::vector;
using std::cout;
using std::endl;


///////////////////////////////////////////////////////////////
// Extern Variables
///////////////////////////////////////////////////////////////

extern iglViewer viewer;
extern igl::opengl::glfw::imgui::ImGuiMenu menu;

extern int AnimateSpeed;

extern bool visibleMechanism;
extern bool visibleAxes;
extern bool visibleGround;

extern bool is_restart;
extern string inputFileName;

extern int frame;
extern bool is_ReadCurve;
extern bool is_ReadMechs;
extern bool is_SaveMechs;
extern bool is_Optimize;
extern libigl_Render myRender;

// to show tips hovering on the UI items
static void HelpMarker(const char* desc);



///=========================================================================================///
///                                      Set Viewer UI
///=========================================================================================///

//// UI main function
void setViewerUI(igl::opengl::glfw::Viewer &viewer)
{
    menu.callback_draw_viewer_window = [&]()
    {
        //// color preset:
        ImGui::GetStyle().Colors[ImGuiCol_WindowBg] = ImVec4(0.9f, 0.9f, 0.9f, 1.00f);
        ImGui::GetStyle().Colors[ImGuiCol_TitleBg] = ImVec4(0.9f, 0.9f, 0.9f, 1.00f);
        ImGui::GetStyle().Colors[ImGuiCol_TitleBgActive] = ImVec4(0.9f, 0.9f, 0.9f, 1.00f);
        ImGui::GetStyle().Colors[ImGuiCol_TitleBgCollapsed] = ImVec4(0.9f, 0.9f, 0.9f, 1.00f);

        ImGui::GetStyle().Colors[ImGuiCol_FrameBg] = ImVec4(1.0f, 1.0f, 1.0f, 1.00f);
        ImGui::GetStyle().Colors[ImGuiCol_FrameBgHovered] = ImVec4(1.0f, 1.0f, 1.0f, 0.50f);
        ImGui::GetStyle().Colors[ImGuiCol_FrameBgActive] = ImVec4(1.0f, 1.0f, 1.0f, 0.50f);

        ImGui::GetStyle().Colors[ImGuiCol_MenuBarBg] = ImVec4(0.9f, 0.9f, 0.9f, 1.00f);
        ImGui::GetStyle().Colors[ImGuiCol_Text] = ImVec4(0.0f, 0.0f, 0.0f, 1.00f);

        ImGui::GetStyle().Colors[ImGuiCol_PopupBg] = ImVec4(1.0f, 1.0f, 1.0f, 1.00f);


        //ImGui::TextWrapped();
        //// Define window position + size
        float menu_width = 240.f * menu.menu_scaling();

        //// Warning: do get the true windows width to relocate the menu, the viewer using highdpi (see Viewer.cpp to support highdpi display）
        int width_window, height_window;
        glfwGetWindowSize(viewer.window,&width_window, &height_window);

        ImGui::SetNextWindowPos(ImVec2(width_window - menu_width, 0.0f),ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSizeConstraints(ImVec2(menu_width, -1.0f), ImVec2(menu_width, -1.0f));
        bool _viewer_menu_visible = true;
        ImGui::Begin(
                "Control Panel", &_viewer_menu_visible,
                ImGuiWindowFlags_NoSavedSettings
        );
        ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.4f);
        if (menu.callback_draw_viewer_menu) { menu.callback_draw_viewer_menu(); }
        ImGui::PopItemWidth();
        ImGui::End();
    };


    // Add content to the default menu window
    menu.callback_draw_viewer_menu = [&]()
    {

        //// global styles of UI
        float w = ImGui::GetContentRegionAvailWidth();
        float p = ImGui::GetStyle().FramePadding.x;
        float gap_between_controlGroups = 4.0f;

        float button_verticalGap = 2.0f;
        float button_horizontalGap = 4*p;
        float button_width = (w-button_horizontalGap)/2.f;

        ///////////////////////////////////////////////////////////////////////////
        //// Optimization Process
        ///////////////////////////////////////////////////////////////////////////

        if (ImGui::CollapsingHeader("Optimization Process", ImGuiTreeNodeFlags_DefaultOpen))
        {
            float head_scale = 1.3f;
            float gap_between_paraGroups = 8.0f;
            float half_width = (w - p) / 2.f;

            ImGui::Dummy(ImVec2(0.0f, 2.0f));

            if(ImGui::Button("Read Input Curve", ImVec2(button_width, 0)))
            {
                inputFileName = igl::file_dialog_open();
                if(inputFileName.empty())
                {
                    cout<< " input is empty "<<endl;
                    return ;
                }
                is_ReadMechs = false;
                is_ReadCurve = true;
                frame = 0;
            }
            ImGui::SameLine(0, button_horizontalGap);
            if(ImGui::Button("Optimization", ImVec2(button_width, 0)))
            {
                is_Optimize = true;
                frame = 0;
            }
            ImGui::Dummy(ImVec2(0.0f, gap_between_paraGroups));
        }

        ImGui::Dummy(ImVec2(0.0f, gap_between_controlGroups));

        ///////////////////////////////////////////////////////////////////////////
        //// I/O files
        ///////////////////////////////////////////////////////////////////////////

        if (ImGui::CollapsingHeader("I/O files", ImGuiTreeNodeFlags_DefaultOpen))
        {
            float head_scale = 1.3f;
            float gap_between_paraGroups = 8.0f;
            float half_width = (w - p) / 2.f;

            ImGui::Dummy(ImVec2(0.0f, 2.0f));

            if(ImGui::Button("Read CamLinkage", ImVec2(button_width, 0)))
            {
                inputFileName = igl::file_dialog_open();
                if(inputFileName.empty())
                {
                    cout<< " input is empty "<<endl;
                    return ;
                }
                is_ReadMechs = true;
                is_ReadCurve = false;
                frame = 0;
            }
            ImGui::SameLine(0, button_horizontalGap);
            ///save mat.dat
            /*if(ImGui::Button("Save CamLinkage", ImVec2(button_width, 0)))
            {
                inputFileName = igl::file_dialog_open();
                if(inputFileName.empty())
                {
                    cout<< " input is empty "<<endl;
                    return ;
                }
                is_SaveMechs = true;
                frame = 0;
            }
             */
            ImGui::Dummy(ImVec2(0.0f, gap_between_paraGroups));
        }

        ImGui::Dummy(ImVec2(0.0f, gap_between_controlGroups));


        ///////////////////////////////////////////////////////////////////////////
        //// Operation Control
        ///////////////////////////////////////////////////////////////////////////

        if (ImGui::CollapsingHeader("Operation Control", ImGuiTreeNodeFlags_DefaultOpen))
        {
            //// gap between the button group and head
            ImGui::Dummy(ImVec2(0.0f, 2.0f));

            ////////////////////////////////////////////////////////////////////
            //// model&shell&parts related
            //// buttons for model/shell IO/generate

            if (ImGui::Button("Stop Motion", ImVec2(button_width, 0)))
            {
                viewer.core().is_animating = false;
            }
            ImGui::SameLine(0, button_horizontalGap);

            if (ImGui::Button("Restart Motion", ImVec2(button_width, 0)))
            {
                viewer.core().is_animating = true;
                is_restart = true;
            }
            ImGui::Dummy(ImVec2(0.0f, button_verticalGap));

            if(ImGui::SliderInt("Animation Speed", &AnimateSpeed, 1, 8))
            {
                cout<< "speed is modified to " <<  AnimateSpeed << endl;
            }
            ImGui::Dummy(ImVec2(0.0f, button_verticalGap));
        }

        ImGui::Dummy(ImVec2(0.0f, gap_between_controlGroups));


        ///////////////////////////////////////////////////////////////////////////
        //// Render Control
        ///////////////////////////////////////////////////////////////////////////

        if (ImGui::CollapsingHeader("Render Control", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::Dummy(ImVec2(0.0f, 2.0f));
            float head_scale = 1.3f;
            float gap_between_renderGroups = 4.0f;
            float half_width = (w - p) / 2.f;
            float transparency;


            ImGui::Dummy(ImVec2(0.0f, gap_between_renderGroups));
            ImGui::SetWindowFontScale(1);

            ImGui::Text("show myModels");
            ImGui::SameLine(half_width, p);
            ImGui::Checkbox("##show myModels", &visibleMechanism);

            ImGui::Text("show ground");
            ImGui::SameLine(half_width, p);
            ImGui::Checkbox("##show ground", &visibleGround);

            ImGui::Text("show axes");
            ImGui::SameLine(half_width, p);
            ImGui::Checkbox("##show axes", &visibleAxes);

            //// transparency needs new shader support
//            ImGui::Text("transparency");
//            HelpMarker("adjust alpha tunnel");
//            ImGui::SameLine(half_width, p);
//            ImGui::SetNextItemWidth(half_width);
//            if (ImGui::DragFloat("##Alpha", &transparency, 0.005f, 0.0f, 1.0f))
//            {
//                //is_render_content_changed = true;
//            }
            ImGui::Dummy(ImVec2(0.0f, gap_between_renderGroups));

        }
        ImGui::Dummy(ImVec2(0.0f, gap_between_controlGroups));
    };

    viewer.plugins.push_back(&menu);
    viewer.data().face_based = true;
    viewer.core().background_color.setConstant(1.0f);
}


static void HelpMarker(const char* desc)
{
    if (ImGui::IsItemHovered())
    {
        ImGui::BeginTooltip();
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
        ImGui::TextUnformatted(desc);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}
