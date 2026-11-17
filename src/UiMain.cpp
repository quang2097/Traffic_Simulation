#include "traffic/SimulationView.hpp"
#include "imgui.h"
#include "imgui_impl_win32.h"
#include "imgui_impl_opengl3.h"
#include <windows.h>
#include <GL/gl.h>
#include <chrono>
#include <exception>
#include <filesystem>

extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND,UINT,WPARAM,LPARAM);
static LRESULT WINAPI windowProcedure(HWND window,UINT message,WPARAM w,LPARAM l) {
    if (ImGui_ImplWin32_WndProcHandler(window,message,w,l)) return true;
    if (message==WM_DESTROY) { PostQuitMessage(0); return 0; }
    if (message==WM_SYSCOMMAND && (w&0xfff0)==SC_KEYMENU) return 0;
    return DefWindowProcW(window,message,w,l);
}
int main(int argc,char** argv) {
    // Desktop launches may inherit an unrelated working directory.
    if(argc==1 && !std::filesystem::exists("examples/annotated-map.png")) {
        wchar_t executable[32768];
        DWORD count=GetModuleFileNameW(nullptr,executable,32768);
        if(count && count<32768) {
            auto directory=std::filesystem::path(executable).parent_path();
            if(std::filesystem::exists(directory/"examples/annotated-map.png"))
                std::filesystem::current_path(directory);
        }
    }
    ImGui_ImplWin32_EnableDpiAwareness();
    WNDCLASSW wc{}; wc.style=CS_OWNDC; wc.lpfnWndProc=windowProcedure;
    wc.hInstance=GetModuleHandleW(nullptr); wc.lpszClassName=L"TrafficSimulation";
    wc.hCursor=LoadCursor(nullptr,IDC_ARROW);
    if (!RegisterClassW(&wc)) return 1;
    HWND window=CreateWindowW(wc.lpszClassName,L"Traffic Lab",WS_OVERLAPPEDWINDOW,
        CW_USEDEFAULT,CW_USEDEFAULT,1400,950,nullptr,nullptr,wc.hInstance,nullptr);
    if (!window) { UnregisterClassW(wc.lpszClassName,wc.hInstance); return 1; }
    HDC dc=GetDC(window);
    PIXELFORMATDESCRIPTOR pfd{}; pfd.nSize=sizeof(pfd); pfd.nVersion=1;
    pfd.dwFlags=PFD_DRAW_TO_WINDOW|PFD_SUPPORT_OPENGL|PFD_DOUBLEBUFFER;
    pfd.iPixelType=PFD_TYPE_RGBA; pfd.cColorBits=32;
    int format=ChoosePixelFormat(dc,&pfd);
    HGLRC context=nullptr;
    bool deviceReady=format && SetPixelFormat(dc,format,&pfd);
    if (deviceReady) context=wglCreateContext(dc);
    if (!context || !wglMakeCurrent(dc,context)) {
        MessageBoxW(window,L"Could not initialize OpenGL.",L"Traffic Lab",MB_ICONERROR);
        if (context) wglDeleteContext(context);
        ReleaseDC(window,dc); DestroyWindow(window); UnregisterClassW(wc.lpszClassName,wc.hInstance); return 1;
    }
    IMGUI_CHECKVERSION(); ImGui::CreateContext();
    ImGui::GetIO().IniFilename=nullptr;
    ImGui::GetIO().ConfigFlags|=ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();
    auto& style=ImGui::GetStyle(); style.FrameRounding=5; style.WindowRounding=0;
    style.FontSizeBase=17;
    style.FramePadding=ImVec2(8,5); style.ItemSpacing=ImVec2(9,7);
    ImGui_ImplWin32_InitForOpenGL(window);
    if (!ImGui_ImplOpenGL3_Init("#version 130")) {
        ImGui_ImplWin32_Shutdown(); ImGui::DestroyContext();
        wglMakeCurrent(nullptr,nullptr); wglDeleteContext(context); ReleaseDC(window,dc);
        DestroyWindow(window); UnregisterClassW(wc.lpszClassName,wc.hInstance); return 1;
    }
    // Optional GPU presentation synchronization. Frame pacing below also covers drivers without it.
    using SwapInterval=BOOL(WINAPI*)(int);
    if (auto swap=reinterpret_cast<SwapInterval>(wglGetProcAddress("wglSwapIntervalEXT"))) swap(1);
    ShowWindow(window,SW_SHOWDEFAULT);
    int result=0;
    try {
        traffic::SimulationView view(argc>1?argv[1]:nullptr);
        auto previous=std::chrono::steady_clock::now();
        bool done=false;
        while (!done) {
            auto frameStart=std::chrono::steady_clock::now();
            MSG message;
            while (PeekMessageW(&message,nullptr,0,0,PM_REMOVE)) {
                TranslateMessage(&message); DispatchMessageW(&message);
                if (message.message==WM_QUIT) done=true;
            }
            if (done) break;
            if (IsIconic(window)) { Sleep(20); previous=std::chrono::steady_clock::now(); continue; }
            double dt=std::chrono::duration<double>(frameStart-previous).count(); previous=frameStart;
            ImGui_ImplOpenGL3_NewFrame(); ImGui_ImplWin32_NewFrame(); ImGui::NewFrame();
            view.frame(dt);
            ImGui::Render();
            RECT rect; GetClientRect(window,&rect); glViewport(0,0,rect.right,rect.bottom);
            glClearColor(0.06f,0.08f,0.11f,1); glClear(GL_COLOR_BUFFER_BIT);
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData()); SwapBuffers(dc);
            auto milliseconds=std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-frameStart).count();
            auto frameBudget=view.isAdvancing()?8:66; // Idle/step mode redraws at ~15 Hz.
            if (milliseconds<frameBudget) Sleep(static_cast<DWORD>(frameBudget-milliseconds));
        }
    } catch (const std::exception& error) {
        MessageBoxA(window,error.what(),"Traffic Lab",MB_ICONERROR); result=1;
    }
    ImGui_ImplOpenGL3_Shutdown(); ImGui_ImplWin32_Shutdown(); ImGui::DestroyContext();
    wglMakeCurrent(nullptr,nullptr); wglDeleteContext(context); ReleaseDC(window,dc);
    DestroyWindow(window); UnregisterClassW(wc.lpszClassName,wc.hInstance);
    return result;
}
