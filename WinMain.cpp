// Hillsen.cpp : Defines the entry point for the application.
//

#include "stdafx.h"
#include "WinMain.h"
#include "Hillsen.h"
#include <Commdlg.h>

Hill *pHills = nullptr;
Hill *Start_Hills();
void Build_Hills(Hill *pH);
void StartHillThread(Hill *pH);
void StartVoxVixThread(Hill *pH);

#define MAX_LOADSTRING 100
#define IDT_TIMER1 90094

// Global Variables:
HINSTANCE hInst;								// current instance
TCHAR szTitle[MAX_LOADSTRING];					// The title bar text
TCHAR szWindowClass[MAX_LOADSTRING];			// the main window class name

void Test_Hills();
HWND hWnd;
HANDLE hThreadHills = 0;
HANDLE hThreadViz = 0;
HBITMAP hVoxVizMap = 0;
HBITMAP hVoxVizDraw = 0;

short mouse_x, mouse_y;

#pragma optimize("", off) 
// Forward declarations of functions included in this code module:
ATOM				MyRegisterClass(HINSTANCE hInstance);
BOOL				InitInstance(HINSTANCE, int);
LRESULT CALLBACK	WndProc(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK	About(HWND, UINT, WPARAM, LPARAM);

int APIENTRY _tWinMain(_In_ HINSTANCE hInstance,
                     _In_opt_ HINSTANCE hPrevInstance,
                     _In_ LPTSTR    lpCmdLine,
                     _In_ int       nCmdShow)
{
	UNREFERENCED_PARAMETER(hPrevInstance);
	UNREFERENCED_PARAMETER(lpCmdLine);

 	// TODO: Place code here.
	MSG msg;
	HACCEL hAccelTable;

	// Initialize global strings
	LoadString(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
	LoadString(hInstance, IDC_HILLSEN, szWindowClass, MAX_LOADSTRING);
	MyRegisterClass(hInstance);

	// Perform application initialization:
	if (!InitInstance (hInstance, nCmdShow))
	{
		return FALSE;
	}

	hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_HILLSEN));

	Hill *pH = Start_Hills();
	pHills = pH;
	StartHillThread(pH);
//	StartVoxVixThread(pH);
//	Test_Hills();



	// Main message loop:
	while (GetMessage(&msg, NULL, 0, 0))
	{
		if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
	}

	return (int) msg.wParam;
}



//
//  FUNCTION: MyRegisterClass()
//
//  PURPOSE: Registers the window class.
//
ATOM MyRegisterClass(HINSTANCE hInstance)
{
	WNDCLASSEX wcex;

	wcex.cbSize = sizeof(WNDCLASSEX);

	wcex.style			= CS_HREDRAW | CS_VREDRAW;
	wcex.lpfnWndProc	= WndProc;
	wcex.cbClsExtra		= 0;
	wcex.cbWndExtra		= 0;
	wcex.hInstance		= hInstance;
	wcex.hIcon			= LoadIcon(hInstance, MAKEINTRESOURCE(IDI_HILLSEN));
	wcex.hCursor		= LoadCursor(NULL, IDC_ARROW);
	wcex.hbrBackground	= (HBRUSH)(COLOR_WINDOW+1);
	wcex.lpszMenuName	= MAKEINTRESOURCE(IDC_HILLSEN);
	wcex.lpszClassName	= szWindowClass;
	wcex.hIconSm		= LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_SMALL));

	return RegisterClassEx(&wcex);
}

//
//   FUNCTION: InitInstance(HINSTANCE, int)
//
//   PURPOSE: Saves instance handle and creates main window
//
//   COMMENTS:
//
//        In this function, we save the instance handle in a global variable and
//        create and display the main program window.
//
BOOL InitInstance(HINSTANCE hInstance, int nCmdShow)
{

   hInst = hInstance; // Store instance handle in our global variable

   hWnd = CreateWindow(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW,
      CW_USEDEFAULT, 0, CW_USEDEFAULT, 0, NULL, NULL, hInstance, NULL);

   if (!hWnd)
   {
      return FALSE;
   }

   ShowWindow(hWnd, nCmdShow);
   UpdateWindow(hWnd);

   SetTimer(hWnd,             // handle to main window 
			IDT_TIMER1,            // timer identifier 
			1000,                 // 10-second interval 
			(TIMERPROC)NULL);     // no timer callback 


   return TRUE;
}

bool isVizVoxing = false;


static int vizWidth = 1024;
static int vizHeight = 720;
static double vizDepth = 6000.0;
static double vizUp = 0.0;
static double vizRight = 0.0;
static double vizAng = 0.0;
static double vizPitch = M_PI / 6.0;
static double vizFov = 0.5;

static void *ThreadVox(void *param)
{
	Hill *pH = (Hill*)param;

	char bmimem[sizeof(BITMAPINFOHEADER) + sizeof(RGBQUAD) * 256];
	BITMAPINFO *bmi = (BITMAPINFO*)bmimem;
	BITMAPINFOHEADER &bih = bmi->bmiHeader;
	bih.biSize = sizeof(BITMAPINFOHEADER);
	bih.biWidth = vizWidth & (~3UL);
	bih.biHeight = -vizHeight;
	bih.biPlanes = 1;
	bih.biBitCount = 24;
	bih.biCompression = BI_RGB;
	bih.biSizeImage = 0;
	bih.biXPelsPerMeter = 14173;
	bih.biYPelsPerMeter = 14173;
	bih.biClrUsed = 0;
	bih.biClrImportant = 0;

	void *Pixels = NULL;
	HBITMAP hbmp = CreateDIBSection(GetDC(hWnd), bmi, DIB_RGB_COLORS, &Pixels, NULL, 0);
	if (Pixels) {
		pH->VoxelView((uint8_t*)Pixels, vizWidth & (~3UL), vizHeight, vizDepth, vizRight, vizUp, vizFov, vizAng, vizPitch);
		hVoxVizMap = hbmp;
		InvalidateRect(hWnd, NULL, TRUE);
	}
	hThreadViz = 0;
	return nullptr;
}

static void *ThreadGo(void *param)
{
	Build_Hills((Hill*)param);
	hThreadHills = 0;
	return NULL;
}


void StartHillThread(Hill *pH)
{
	hThreadHills = CreateThread(NULL, 256*1024, (LPTHREAD_START_ROUTINE)ThreadGo, pH,
							  0, NULL);
}

void StartVoxVixThread(Hill *pH)
{
	hThreadViz = CreateThread(NULL, 16384, (LPTHREAD_START_ROUTINE)ThreadVox, pH,
								0, NULL);

}


bool bLeftButtonDown = false;
bool bRightButtonDown = false;

double FixAng(double a)
{
	a = fmod(a + 3 * M_PI, 2 * M_PI);
	return a-M_PI;
}
#include <comdef.h> 
#include <shlobj.h>

//_COM_SMARTPTR_TYPEDEF(IFileOpenDialog, __uuidof(IFileOpenDialog));
//_COM_SMARTPTR_TYPEDEF(IShellItem, __uuidof(IShellItem));

//#include <sstream> 
//using std::endl;
//using std::wostringstream;
//
//  FUNCTION: WndProc(HWND, UINT, WPARAM, LPARAM)
//
//  PURPOSE:  Processes messages for the main window.
//
//  WM_COMMAND	- process the application menu
//  WM_PAINT	- Paint the main window
//  WM_DESTROY	- post a quit message and return
//
//
LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	int wmId, wmEvent;
	PAINTSTRUCT ps;
	HDC hdc;

	switch (message)
	{
	case WM_COMMAND:
		wmId    = LOWORD(wParam);
		wmEvent = HIWORD(wParam);
		// Parse the menu selections:
		switch (wmId)
		{
		case IDM_ABOUT:
			DialogBox(hInst, MAKEINTRESOURCE(IDD_ABOUTBOX), hWnd, About);
			break;
		case IDM_EXIT:
			DestroyWindow(hWnd);
			break;
		case ID_FILE_LOAD: {
			OPENFILENAME ofn = { 0 };
			wchar_t szFileName[MAX_PATH] = L"";

			ofn.lStructSize = sizeof(ofn); // SEE NOTE BELOW
			ofn.hwndOwner = hWnd;
			ofn.lpstrFilter = L"Hill Files (*.hill)\0*.hill\0All Files (*.*)\0*.*\0";
			ofn.lpstrFile = szFileName;
			ofn.nMaxFile = MAX_PATH;
			ofn.Flags = OFN_EXPLORER | OFN_FILEMUSTEXIST | OFN_HIDEREADONLY|OFN_ALLOWMULTISELECT;
			ofn.lpstrDefExt = L"txt";

			if (GetOpenFileName(&ofn)) {
				FILE *f;
				if (_wfopen_s(&f, szFileName, L"rb") == 0) {
					int width, height;
					if (fread(&width, sizeof(width), 1, f) == 1 &&
						fread(&height, sizeof(height), 1, f) == 1) {
						if (width == pHills->w && height == pHills->d) {
							fread(pHills->hgt, width * height * sizeof(double), 1, f);
						} else if (hThreadHills == 0 && hThreadViz == 0) {
							double *data = (double*)malloc(width * height * sizeof(double));
							fread(data, width * height * sizeof(double), 1, f);
							Hill *old = pHills;
							while (hThreadViz != 0)
								Sleep(1);
							pHills = new Hill(width, height);
							pHills->hgt = data;
							free(old->hgt);
							free(old);
						}
					}
					fclose(f);
				}
			}
			break;
		}
		case ID_FILE_SAVE: {
			OPENFILENAME ofn = { 0 };
			wchar_t szFileName[MAX_PATH] = L"";

			ofn.lStructSize = sizeof(ofn); // SEE NOTE BELOW
			ofn.hwndOwner = hWnd;
			ofn.lpstrFilter = L"Hill Files (*.hill)\0*.hill\0All Files (*.*)\0*.*\0";
			ofn.lpstrFile = szFileName;
			ofn.nMaxFile = MAX_PATH;
			ofn.Flags = OFN_EXPLORER | OFN_FILEMUSTEXIST | OFN_HIDEREADONLY|OFN_ALLOWMULTISELECT;
			ofn.lpstrDefExt = L"txt";
			if (GetSaveFileName(&ofn)) {
				FILE *f;
				if (_wfopen_s(&f, szFileName, L"wb") == 0) {
					fwrite(&pHills->w, sizeof(pHills->w), 1, f);
					fwrite(&pHills->d, sizeof(pHills->d), 1, f);
					fwrite(pHills->hgt, pHills->w * pHills->d * sizeof(double), 1, f);
					fclose(f);
				}

			}
			break;
		}
		case ID_FILE_GENERATE:
			if (hThreadHills == 0) {
				pHills->stop = false;
				StartHillThread(pHills);
			}
			break;
		case ID_FILE_STOP:
			if (pHills)
				pHills->stop = true;
			break;
		default:
			return DefWindowProc(hWnd, message, wParam, lParam);
		}
		break;
	case WM_TIMER:
		if (hThreadViz == 0 && hVoxVizMap == 0 && !bLeftButtonDown && !bRightButtonDown) {
			RECT rect;
			GetClientRect(hWnd, &rect);
			vizWidth = (rect.right - rect.left);
			vizHeight = (rect.bottom - rect.top);
			StartVoxVixThread(pHills);
		}
		break;

	case WM_LBUTTONDOWN:
		mouse_x = short(lParam);
		mouse_y = short(lParam>>16);
		bLeftButtonDown = true;
		break;

	case WM_LBUTTONUP:
		bLeftButtonDown = false;
		break;

	case WM_RBUTTONDOWN:
		mouse_x = short(lParam);
		mouse_y = short(lParam>>16);
		bRightButtonDown = true;
		break;

	case WM_RBUTTONUP:
		bRightButtonDown = false;
		break;

	case WM_MOUSELEAVE:
		bLeftButtonDown = false;
		bRightButtonDown = false;
		break;

	case WM_MOUSEMOVE: {

		RECT rect;
		GetClientRect(hWnd, &rect);
		short new_mouse_x = short(lParam);
		short new_mouse_y = short(lParam>>16);
		short delta_mouse_x = new_mouse_x - mouse_x;
		short delta_mouse_y = new_mouse_y - mouse_y;
		mouse_x = new_mouse_x;
		mouse_y = new_mouse_y;

		// test hit / proj
		if (pHills) {
			Pos2 pos = pHills->ScreenHit(rect.right-rect.left, rect.bottom-rect.top,
										 mouse_x, mouse_y, vizDepth, vizRight, vizUp, vizFov, vizAng, vizPitch);
			if (pos.x >= 0) {
				Pos2 bkp = pHills->ScreenProj(rect.right-rect.left, rect.bottom-rect.top,
											  pos.x, pos.y, vizDepth, vizRight, vizUp, vizFov, vizAng, vizPitch);
			}
			//			Pos2 ScreenProj(int width, int height, int x, int y, double cam_offs, double cam_side, double cam_up, double tan_fov, double ang, double pitch);
			//			Pos2 ScreenHit(int width, int height, int x, int y, double cam_offs, double cam_side, double cam_up, double tan_fov, double ang, double pitch);
		}


		if (bLeftButtonDown && !(wParam & (MK_CONTROL | MK_SHIFT))) {
			double rel_x = double(delta_mouse_x) / double(rect.right - rect.left);
			double rel_y = double(delta_mouse_y) / double(rect.bottom - rect.top);
			vizAng = FixAng(vizAng + rel_x * 2.0 * M_PI);
			vizPitch = FixAng(vizPitch + rel_y * M_PI_2);
			if (hThreadViz == 0 && hVoxVizMap == 0) {
				vizWidth = (rect.right - rect.left)>>1;
				vizHeight = (rect.bottom - rect.top)>>1;
				StartVoxVixThread(pHills);
			}
		} else if (wParam & MK_SHIFT) {
			double rel_y = double(delta_mouse_y) / double(rect.bottom - rect.top);
			vizDepth += rel_y * 2000.0;
			if (hThreadViz == 0 && hVoxVizMap == 0) {
				vizWidth = (rect.right - rect.left)>>1;
				vizHeight = (rect.bottom - rect.top)>>1;
				StartVoxVixThread(pHills);
			}
		} else if (bLeftButtonDown || bRightButtonDown || (wParam & MK_CONTROL)) {
			double rel_x = double(delta_mouse_x) / double(rect.right - rect.left);
			double rel_y = double(delta_mouse_y) / double(rect.bottom - rect.top);
			vizRight -= rel_x * 500.0;// vizDepth * 2.0 / (rect.right-rect.left) / vizFov;
			vizUp += rel_y * 500.0;// vizDepth * 2.0 / (rect.right-rect.left) / vizFov;
			if (hThreadViz == 0 && hVoxVizMap == 0) {
				vizWidth = (rect.right - rect.left)>>1;
				vizHeight = (rect.bottom - rect.top)>>1;
				StartVoxVixThread(pHills);
			}
		}

		break;
	}

	case WM_MOUSEWHEEL: {
		int delta = GET_WHEEL_DELTA_WPARAM(wParam);
		vizDepth += delta / 10.0;
		break;
	}

	case WM_ERASEBKGND:
		return 0;

	case WM_PAINT:
		hdc = BeginPaint(hWnd, &ps);
		if (hVoxVizMap || hVoxVizDraw) {
			if (hVoxVizMap) {
				if (hVoxVizDraw)
					DeleteObject(hVoxVizDraw);
				hVoxVizDraw = hVoxVizMap;
				hVoxVizMap = 0;
			}

			RECT rect;
			GetClientRect(hWnd, &rect);

			BITMAP bm;

			GetObject(hVoxVizDraw, sizeof(BITMAP), &bm);

			HDC hdc_tmp = CreateCompatibleDC(hdc);
			HGDIOBJ prevBiMap = SelectObject(hdc_tmp, hVoxVizDraw);
			int dw = rect.right - rect.left;
			int dh = rect.bottom - rect.top;;
			int sw = bm.bmWidth;
			int sh = bm.bmHeight;

			StretchBlt(hdc, 0, 0, dw, dh, hdc_tmp, 0, 0, sw, sh, SRCCOPY);
			SelectObject(hdc_tmp, prevBiMap);

			Pos2 pos = pHills->ScreenHit(rect.right-rect.left, rect.bottom-rect.top,
										 mouse_x, mouse_y, vizDepth, vizRight, vizUp, vizFov, vizAng, vizPitch);
			if (pos.x >= 0) {
				Pos2 bkp = pHills->ScreenProj(rect.right-rect.left, rect.bottom-rect.top,
											  pos.x, pos.y, vizDepth, vizRight, vizUp, vizFov, vizAng, vizPitch);
				if (bkp.x >= 4 && bkp.y >= 4 && bkp.x < (rect.right-rect.left-4) && bkp.y < (rect.bottom-rect.top-4)) {
					HPEN hPen = CreatePen(PS_DOT, 1, RGB(255, 255, 255));
					HGDIOBJ hPenPrev = SelectObject(hdc, hPen);
					Ellipse(hdc, bkp.x-4, bkp.y-4, bkp.x+4, bkp.y+4);
					SelectObject(hdc, hPenPrev);
				}
			}


		}

		// TODO: Add any drawing code here...
		EndPaint(hWnd, &ps);
		break;
	case WM_DESTROY:
		PostQuitMessage(0);
		break;
	default:
		return DefWindowProc(hWnd, message, wParam, lParam);
	}
	return 0;
}

// Message handler for about box.
INT_PTR CALLBACK About(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
	UNREFERENCED_PARAMETER(lParam);
	switch (message)
	{
	case WM_INITDIALOG:
		return (INT_PTR)TRUE;

	case WM_COMMAND:
		if (LOWORD(wParam) == IDOK || LOWORD(wParam) == IDCANCEL)
		{
			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		}
		break;
	}
	return (INT_PTR)FALSE;
}
