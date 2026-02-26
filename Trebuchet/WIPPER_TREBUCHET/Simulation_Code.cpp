#include <windows.h>
#include <gdiplus.h>
#include <sstream>
#include <iomanip>
#include "SIMULATION_ENGINE.h"

#pragma comment(lib, "gdiplus.lib")
#pragma comment(lib, "user32.lib")
#pragma comment(lib, "gdi32.lib")

using namespace Gdiplus;

// Global variables
TrebuchetSimulation* g_simulation = nullptr;
HWND g_hwnd = nullptr;
bool g_paused = false;
int g_scale = 300; // pixels per meter
int g_offset_x = 100;
int g_offset_y = 400;

// Function prototypes
LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam);
void DrawTrebuchet(Graphics& graphics);
void DrawProjectile(Graphics& graphics);
void DrawDebugInfo(Graphics& graphics);

int APIENTRY WinMain(HINSTANCE hInstance, HINSTANCE, LPSTR, int nCmdShow) {
    GdiplusStartupInput gdiplusStartupInput;
    ULONG_PTR gdiplusToken;
    GdiplusStartup(&gdiplusToken, &gdiplusStartupInput, nullptr);

    // Register window class - Use ANSI version
    const char CLASS_NAME[] = "TrebuchetSimulationClass";

    WNDCLASSA wc = {};
    wc.lpfnWndProc = WndProc;
    wc.hInstance = hInstance;
    wc.lpszClassName = CLASS_NAME;
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);

    RegisterClassA(&wc);

    // Create window - Use ANSI version
    g_hwnd = CreateWindowExA(
        0,
        CLASS_NAME,
        "Trebuchet Simulation Visualizer",
        WS_OVERLAPPEDWINDOW,
        CW_USEDEFAULT, CW_USEDEFAULT, 1200, 800,
        nullptr,
        nullptr,
        hInstance,
        nullptr
    );

    if (!g_hwnd) return -1;

    // Initialize simulation
    g_simulation = new TrebuchetSimulation();

    ShowWindow(g_hwnd, nCmdShow);
    UpdateWindow(g_hwnd);

    // Main message loop with simulation stepping
    MSG msg = {};
    while (GetMessage(&msg, nullptr, 0, 0)) {
        TranslateMessage(&msg);
        DispatchMessage(&msg);

        // Step simulation if not paused
        if (!g_paused && g_simulation->IsSimulationRunning()) {
            for (int i = 0; i < 50; ++i) {  // Multiple steps per frame
                g_simulation->Step();
            }
            InvalidateRect(g_hwnd, nullptr, FALSE);
        } else if (g_paused && !g_simulation->IsSimulationRunning()) {
            InvalidateRect(g_hwnd, nullptr, FALSE);
        }
    }

    delete g_simulation;
    GdiplusShutdown(gdiplusToken);
    return (int)msg.wParam;
}

LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    switch (msg) {
        case WM_PAINT: {
            PAINTSTRUCT ps;
            HDC hdc = BeginPaint(hwnd, &ps);

            Graphics graphics(hdc);
            graphics.SetSmoothingMode(SmoothingModeAntiAlias);

            // Clear background
            SolidBrush brush(Color(255, 240, 240, 240));
            graphics.FillRectangle(&brush, ps.rcPaint.left, (INT)ps.rcPaint.top,
                                   (INT)(ps.rcPaint.right - ps.rcPaint.left),
                                   (INT)(ps.rcPaint.bottom - ps.rcPaint.top));

            // Draw ground
            Pen groundPen(Color(0, 0, 0), 3);
            graphics.DrawLine(&groundPen, 0, g_offset_y, 1200, g_offset_y);

            // Draw trebuchet and projectile
            DrawTrebuchet(graphics);
            DrawProjectile(graphics);
            DrawDebugInfo(graphics);

            EndPaint(hwnd, &ps);
            return 0;
        }

        case WM_KEYDOWN: {
            if (wParam == VK_SPACE) {
                g_paused = !g_paused;
            } else if (wParam == 'R') {
                delete g_simulation;
                g_simulation = new TrebuchetSimulation();
                g_paused = false;
            }
            InvalidateRect(hwnd, nullptr, FALSE);
            return 0;
        }

        case WM_DESTROY:
            PostQuitMessage(0);
            return 0;
    }

    return DefWindowProc(hwnd, msg, wParam, lParam);
}

void DrawTrebuchet(Graphics& graphics) {
    if (!g_simulation) return;

    double theta_main = g_simulation->theta_main;
    double theta_cw_rel = g_simulation->theta_cw_rel;
    double theta_cw_abs = theta_main + theta_cw_rel;

    // Main pivot at origin
    INT pivot_x = g_offset_x;
    INT pivot_y = g_offset_y;

    // Main arm endpoint
    INT arm_x = pivot_x + (INT)(g_scale * g_simulation->r_main_tip * cos(theta_main));
    INT arm_y = pivot_y - (INT)(g_scale * g_simulation->r_main_tip * sin(theta_main));

    // Sling pin (end of main arm)
    INT sling_pin_x = arm_x;
    INT sling_pin_y = arm_y;

    // Projectile position (at end of sling)
    INT projectile_x = sling_pin_x + (INT)(g_scale * g_simulation->sling_length * cos(theta_main));
    INT projectile_y = sling_pin_y - (INT)(g_scale * g_simulation->sling_length * sin(theta_main));

    // Counterweight hinge
    INT hinge_x = pivot_x + (INT)(g_scale * g_simulation->r_pivot_to_hinge * cos(theta_main));
    INT hinge_y = pivot_y - (INT)(g_scale * g_simulation->r_pivot_to_hinge * sin(theta_main));

    // Counterweight position
    INT cw_x = hinge_x + (INT)(g_scale * g_simulation->r_hinge_to_cw_cg * cos(theta_cw_abs));
    INT cw_y = hinge_y - (INT)(g_scale * g_simulation->r_hinge_to_cw_cg * sin(theta_cw_abs));

    // Draw main pivot
    SolidBrush pivotBrush(Color(0, 0, 0));
    graphics.FillEllipse(&pivotBrush, pivot_x - 5, pivot_y - 5, 10, 10);

    // Draw main arm
    Pen armPen(Color(139, 69, 19), 8);
    graphics.DrawLine(&armPen, pivot_x, pivot_y, arm_x, arm_y);

    // Draw sling
    Pen slingPen(Color(100, 100, 100), 2);
    graphics.DrawLine(&slingPen, sling_pin_x, sling_pin_y, projectile_x, projectile_y);

    // Draw projectile
    SolidBrush projectileBrush(Color(255, 0, 0));
    graphics.FillEllipse(&projectileBrush, projectile_x - 5, projectile_y - 5, 10, 10);

    // Draw counterweight arm
    Pen cwArmPen(Color(200, 100, 50), 6);
    graphics.DrawLine(&cwArmPen, pivot_x, pivot_y, hinge_x, hinge_y);
    graphics.DrawLine(&cwArmPen, hinge_x, hinge_y, cw_x, cw_y);

    // Draw counterweight (cylinder representation)
    SolidBrush cwBrush(Color(100, 100, 100));
    graphics.FillEllipse(&cwBrush, cw_x - 8, cw_y - 12, 16, 24);
}

void DrawProjectile(Graphics& graphics) {
    if (!g_simulation || g_simulation->trajectory.empty()) return;

    Pen trajPen(Color(255, 100, 100), 2);
    for (size_t i = 1; i < g_simulation->trajectory.size(); ++i) {
        auto& prev = g_simulation->trajectory[i - 1];
        auto& curr = g_simulation->trajectory[i];

        INT x1 = g_offset_x + (INT)(g_scale * prev.x);
        INT y1 = g_offset_y - (INT)(g_scale * prev.y);
        INT x2 = g_offset_x + (INT)(g_scale * curr.x);
        INT y2 = g_offset_y - (INT)(g_scale * curr.y);

        graphics.DrawLine(&trajPen, x1, y1, x2, y2);
    }
}

void DrawDebugInfo(Graphics& graphics) {
    if (!g_simulation) return;

    Font font(L"Arial", 12);
    SolidBrush textBrush(Color(0, 0, 0));

    int y_pos = 20;
    int line_height = 20;

    // Status
    const wchar_t* status_str = L"RUNNING";
    if (g_paused) status_str = L"PAUSED";
    if (g_simulation->has_stalled) status_str = L"STALLED";
    if (g_simulation->is_released) status_str = L"PROJECTILE RELEASED";

    graphics.DrawString(status_str, -1, &font, PointF(20.0f, (REAL)y_pos), &textBrush);
    y_pos += line_height;

    // Simulation data
    wchar_t buffer[256];
    swprintf_s(buffer, sizeof(buffer)/sizeof(wchar_t), L"Main Angle: %.2f°", 
               g_simulation->theta_main * 180.0 / g_simulation->PI);
    graphics.DrawString(buffer, -1, &font, PointF(20.0f, (REAL)y_pos), &textBrush);
    y_pos += line_height;

    swprintf_s(buffer, sizeof(buffer)/sizeof(wchar_t), L"Angular Velocity: %.3f rad/s", 
               g_simulation->omega_main);
    graphics.DrawString(buffer, -1, &font, PointF(20.0f, (REAL)y_pos), &textBrush);
    y_pos += line_height;

    swprintf_s(buffer, sizeof(buffer)/sizeof(wchar_t), L"Iterations: %d", 
               g_simulation->iterations);
    graphics.DrawString(buffer, -1, &font, PointF(20.0f, (REAL)y_pos), &textBrush);
    y_pos += line_height;

    if (g_simulation->is_released) {
        swprintf_s(buffer, sizeof(buffer)/sizeof(wchar_t), L"Release Velocity: %.3f m/s", 
                   g_simulation->v_release);
        graphics.DrawString(buffer, -1, &font, PointF(20.0f, (REAL)y_pos), &textBrush);
        y_pos += line_height;

        swprintf_s(buffer, sizeof(buffer)/sizeof(wchar_t), L"Estimated Range: %.3f m", 
                   g_simulation->range);
        graphics.DrawString(buffer, -1, &font, PointF(20.0f, (REAL)y_pos), &textBrush);
        y_pos += line_height;
    }

    // Instructions
    y_pos = 700;
    graphics.DrawString(L"SPACE: Pause/Resume | R: Reset", -1, &font, 
                       PointF(20.0f, (REAL)y_pos), &textBrush);
}
