#ifndef DRAWING_WINDOW_H
#define DRAWING_WINDOW_H

#include "unicycle_sim.h"
#include "basic_window.h"
#include "resource.h"
#include "winbmp.h"
#include <ShObjIdl.h>
#include <windowsx.h>
#include <d2d1.h>
#include <list>
#include <memory>
#include <math.h>
#pragma comment(lib, "d2d1")

#define S_PI 3.14159265358979323846

class GdiBitmap
{
private:
	// Bmp
	Bmp bitmap;

	// Gdi objects
	HBITMAP hBmp{ NULL };			// Bmp handle
	HDC hdcBmp{ NULL };				// Device context
	BITMAP bmpFileHeader;			// Bitmap header struct
	BITMAPINFOHEADER bmpInfoHeader;	// Bitmap info header struct
	POINT ptParallelogram[3];	
	HBITMAP hBmpMask{ NULL };

	// Device Context Handler
	//HDC hdcScreen;
	//HDC hdcWindow;
	//HDC hdcTemp;

public:
	GdiBitmap() = default;
	GdiBitmap(std::string bmpPath, HWND hwnd) { Load(bmpPath); CreateMemDC(hwnd); }
	~GdiBitmap() { if (hdcBmp) DeleteObject(hdcBmp); }
	BOOL Load(std::string bmpPath);
	BOOL CreateMemDC(HWND hwnd);
	BOOL CaptureScreen(HWND hwnd);
	BOOL Display(HWND hwnd);
	void Draw(HWND hwnd, const POINT& centroidPos = POINT{ 0,0 }, const float& orientation = 0.0);
	void Release();
};

/**
* Interface (abstract class) for drawing shape
**/
template <typename D2D1_SHAPE>
struct IShape
{
	double pi{ 3.14159265358979323846 };
	virtual D2D1_SHAPE& Shape() = 0;					
	virtual D2D1_COLOR_F GetColor() = 0;				
	virtual void SetColor(D2D1_COLOR_F color) = 0;		
	virtual void Draw(ID2D1RenderTarget* pRenderTarget, ID2D1SolidColorBrush* pBrush) = 0;	
	virtual BOOL HitTest(float dipX, float dipY) = 0;	
};

class ColorEllipse : public IShape<D2D1_ELLIPSE>
{
private:
	D2D1_ELLIPSE ellipse;
	D2D1_COLOR_F color;
	float rotation{ 0 }; // degrees

public:
	ColorEllipse() = default;
	ColorEllipse(D2D1_POINT_2F cursorPosition, D2D1_COLOR_F color);
	D2D1_ELLIPSE& Shape() override { return ellipse; }
	D2D1_COLOR_F GetColor() override { return color; }
	float& Rotation() { return rotation; }
	void SetColor(D2D1_COLOR_F color) override { this->color = color; }
	void Draw(ID2D1RenderTarget* pRenderTarget, ID2D1SolidColorBrush* pBrush) override;
	BOOL HitTest(float dipX, float dipY) override;
	void Move(float deltaX, float deltaY);
};

class ColorRectangle : public IShape<D2D1_RECT_F>
{
private:
	D2D1_RECT_F rectangle;
	D2D1_COLOR_F color;
	D2D1_POINT_2F relCornerCenterOffset{ 0, 0 }; // relative to upper-left corner in robot frame
	D2D1_POINT_2F absCornerCenterOffset{ 0, 0 };	// relative to upper-left corner in abs frame
	D2D1_POINT_2F originPosition{ 0,0 };	// position of center of rotation in abs frame
	D2D1_POINT_2F relCornerOffset{ 0,0 };		// Upper-left corner to bottom-right corner
	D2D1_POINT_2F absCornerOffset{ 0,0 };
	float rotation = 0; // degrees
public:
	ColorRectangle() = default;
	ColorRectangle(FLOAT left, FLOAT top, FLOAT right, FLOAT bottom, FLOAT rotation, 
		D2D1_POINT_2F relCornerOffset, D2D1_COLOR_F color);
	ColorRectangle(FLOAT width, FLOAT height, FLOAT rotation, D2D1_POINT_2F originPosition, D2D1_POINT_2F relCornerOffset, D2D1_COLOR_F color);
	ColorRectangle(const ColorRectangle& rect) : rectangle{ rect.rectangle }, color{ rect.color }, rotation{ rect.rotation },
		relCornerCenterOffset{ rect.relCornerCenterOffset }, absCornerCenterOffset{ rect.absCornerCenterOffset } {}
	ColorRectangle(D2D1_POINT_2F cursorPosition, D2D1_COLOR_F color);
	D2D1_RECT_F& Shape() override { return rectangle; }
	D2D1_COLOR_F GetColor() override { return color; }
	float& Rotation() { return rotation; }
	void SetColor(D2D1_COLOR_F color) override { this->color = color; }
	void Draw(ID2D1RenderTarget* pRenderTarget, ID2D1SolidColorBrush* pBrush) override;
	BOOL HitTest(float dipX, float dipY) override;
	void Move(float deltaX, float deltaY, float rotation);
	void UpdateRectangle(D2D1_POINT_2F absPosition, float rotation);
	void Update(D2D1_POINT_2F relCornerCenterOffset);
};

D2D1_POINT_2F Rotate2D(const D2D1_POINT_2F& vector, float rotation);

D2D1_POINT_2F Add2D(const D2D1_POINT_2F& vector1, const D2D1_POINT_2F& vector2);

template <typename Shape>
class ShapeList
{
private:
	std::list<std::shared_ptr<Shape>> shapes;
	std::shared_ptr<Shape> pShape;
	const D2D1::ColorF::Enum colors[6]								
	{
		D2D1::ColorF::Yellow, D2D1::ColorF::Salmon, D2D1::ColorF::LimeGreen,
		D2D1::ColorF::Aqua, D2D1::ColorF::Beige, D2D1::ColorF::Violet
	};
	unsigned long long colorIndex;	

public:
	ShapeList() : shapes{}, colorIndex{ 0 } {}
	std::shared_ptr<Shape> SelectedShape();
	HRESULT InsertShape(float dipX, float dipY);
	BOOL SelectShape(float dipX, float dipY);
	void ClearSelection() { pShape = nullptr; }
	void Draw(ID2D1RenderTarget* pRenderTarget, ID2D1SolidColorBrush* pBrush);
};

enum class Mode
{
	DrawMode,	
	SelectionMode,
	DragMode
};

class DrawingWindow : public BasicWindow<DrawingWindow>
{
	
protected:
	/**
	* Render target is the location where your program draw (window, bitmap, etc.)
	* Device is an abstraction that represents the GPU or CPU
	* Resource is an object used for drawing.
	*	- Brush: control painting of lines and regions
	*	- Stroke style: control appearance of a line
	*	- Geometry: collection of lines and curves
	*	- Mesh: shape formed out of triangles
	*	- Render target
	* Factory is an object that creates other object (render targets and device independent resources)
	* Render target object creates device-dependent resources
	**/
	ID2D1Factory* pFactory;											// Factory object
	ID2D1HwndRenderTarget* pRenderTarget;							// Render target with the application window
	ID2D1SolidColorBrush* pBrush;									// Solid color brush
	D2D1_ELLIPSE ellipse;											// Ellipse
	D2D1_POINT_2F ptMouse;											// Mouse position
	D2D1_POINT_2F drawStartPos{ 0 };
	D2D1_POINT_2F dragObjRelPos{ 0 };
	Mode mode;														// Mode
	HCURSOR hCursor;												// Cursor
	ShapeList<ColorEllipse> ellipses;								// Ellipse list
	ShapeList<ColorRectangle> rectangles;							// Rectangle list

	// Utility
	void SetMode(Mode mode);

	// Graphics
	HRESULT CreateGraphicsResources();
	void DiscardGraphicsResources();
	void Resize();

	// Message handlers
	void OnPaint();												// WM_Paint handler
	void OnLeftButtonDown(int pixelX, int pixelY, DWORD flags);	// WM_LBUTTONDOWN handler
	void OnLeftButtonUp();										// WN_LBUTTONUP handler
	void OnRightButtonDown(int pixelX, int pixelY, DWORD flags);
	void OnRightButtonUp();
	void OnMouseMove(int pixelX, int pixelY, DWORD flags);		// WM_MOUSEMOVE handler
	void OnMouseWheel(int delta);								// WM_MOUSEWHEEL handler
	void OnKeyDown(UINT vkey);									// WM_KEYDOWN handler

	// BitMap
	GdiBitmap bmp;
	void CaptureBitmap();
	void DisplayBitmap();

public:
	DrawingWindow() : pFactory{ NULL }, pRenderTarget{ NULL }, pBrush{ NULL }, 
		ellipse{ D2D1::Ellipse(D2D1::Point2F(), 0, 0) }, ptMouse(D2D1::Point2F()),
		mode{ Mode::DrawMode }, hCursor{} {}
	PCWSTR ClassName() const { return L"Drawing Window Class"; }
	LRESULT HandleMessage(UINT uMsg, WPARAM wParam, LPARAM lParam) override;
};

int DrawingWindowDemo(HINSTANCE hInstance, HINSTANCE hPrevInstance, PWSTR pCmdLine, int nCmdShow);

/**
* Direct2D expect device-independent pixels (DIPs)
* A DPI is defined as 1/96th of a logical inch
* DPI setting: 96, 120, 144
**/
class DPIScale
{
	static float s_scale;
public:
	
	static void Init(HWND hwnd)
	{
		float dpi = (float) GetDpiForWindow(hwnd);
		s_scale = dpi / 96.0f;
	}

	template <typename T>
	static D2D1_POINT_2F PixelsToDIPs(T x, T y)
	{
		return D2D1::Point2F(static_cast<float>(x) / s_scale, static_cast<float>(y) / s_scale);
	}
};

//float DPIScale::s_scale = 1.0f;


#endif

