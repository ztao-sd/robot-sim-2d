#ifndef WINSIM2D_H
#define WINSIM2D_H

#include "drawing_window.h"
#include "unicycle_sim.h"
#include <map>

class Robot2DModel
{
private:
	FLOAT wheelLength = 50.0f;
	FLOAT wheelWidth = 10.0f;
	FLOAT bodyLength = 200.0f;
	FLOAT bodyWidth = 121.0f;
	std::unique_ptr<ColorRectangle> body{ nullptr };
	std::unique_ptr<ColorRectangle> leftWheel{ nullptr };
	std::unique_ptr<ColorRectangle> rightWheel{ nullptr };
	D2D1_POINT_2F absCenterOfRotation;
	D2D1_POINT_2F relBodyOffset{ 30, bodyWidth / 2 };
	D2D1_POINT_2F relLeftWheelOffset{ 30, bodyWidth / 2 + wheelWidth };
	D2D1_POINT_2F relRightWheelOffset{30, -(bodyWidth / 2 + wheelWidth) };
	float rotation;

	void updateParameters(UnicycleRobot& robot);

public:
	FLOAT scale = 1.0f;

	// Interface
	Robot2DModel() = default;
	Robot2DModel(UnicycleRobot& robot);
	void Draw(ID2D1RenderTarget* pRenderTarget, ID2D1SolidColorBrush* pBrush);
	void Move(FLOAT deltaX, FLOAT deltaY, FLOAT rotation);
	void UpdateRectangles(D2D1_POINT_2F position, FLOAT rotation);
	void UpdateState(UnicycleRobot& robot);
	void Scale(UnicycleRobot& robot, float scale);
};

class GDIBmp : public Bmp
{
private:
	// GDI objects
	HBITMAP hBmp{ NULL };			// Bmp handle
	HDC hdcBmp{ NULL };				// Device context
	BITMAP bmpFileHeader;			// Bitmap header struct
	BITMAPINFOHEADER bmpInfoHeader;	// Bitmap info header struct
	POINT ptParallelogram[3];		// Three points to define parallelogram draw area
	// HBITMAP hBmpMask{ NULL };
	void BltTransfer(HWND hwnd, const POINT* ptParallelogram, HBITMAP bMask);
	uint8_t* bmpMask{ nullptr };
	HBITMAP hBmpMask{ NULL };

public:
	GDIBmp() = default;
	GDIBmp(std::string bmpPath, HWND hwnd) { Load(bmpPath); CreateMemDC(hwnd); }
	~GDIBmp() { if (hdcBmp) DeleteObject(hdcBmp); }
	void Load(std::string bmpPath);
	void CreateMemDC(HWND hwnd);
	void Display(HWND hwnd);
	void Draw(HWND hwnd, const POINT& centroidPos = POINT{ 0,0 }, const float& orientation = 0.0, uint8_t* pixelMask = nullptr);
	void CreatePixelMask(uint8_t* pixelMask);
	BOOL CaptureScreen(HWND hwnd);
	char* CaptureWindow(HWND hwnd);
	void Release();

	template <typename F>
	uint8_t*& PixelMask(F&& maskLambda, uint8_t*& pixelMask);
	uint8_t*& BlackPixelMask();
	uint8_t*& RoundPixelMask(float radius, POINT center);
};

class WinSim2D : public BasicWindow<WinSim2D>
{
protected:
	ID2D1Factory* pFactory{NULL};
	ID2D1HwndRenderTarget* pRenderTarget{NULL};
	ID2D1SolidColorBrush* pBrush{NULL};
	
	GDIBmp bmp{};
	std::vector<GDIBmp> obstacleBmps;
	std::vector<GDIBmp> robotBmps;

	HRESULT CreateGraphicsResources();
	void DiscardGraphicsResources();
	void Resize();
	void OnPaint();
public:
	WinSim2D() = default;
	PCWSTR ClassName() const { return L"Window 2D Simulation Class"; }
	LRESULT HandleMessage(UINT uMsg, WPARAM wParam, LPARAM lParam) override;
};

template<typename F>
inline uint8_t*& GDIBmp::PixelMask(F&& maskLambda, uint8_t*& pixelMask)
{
	if (Pixels().IsImage() && !pixelMask)
	{
		const auto height = Pixels().pixelMatrix.size();
		const auto width = Pixels().pixelMatrix.front().size();
		const auto nPlanes = 1;
		const auto bitPerPixel = 1;
		const auto bufferSize = (((width * nPlanes * bitPerPixel + 15) >> 4) << 1) * height;

		pixelMask = new uint8_t[bufferSize];
		int idx = 0;
		uint8_t bitMask = 0x80; // Big-endian (mask)

		for (int row = height - 1; row >= 0; --row)
		{
			*(pixelMask + idx) = 0x00;
			if (bitMask != 0x80)
			{
				bitMask = 0x80;
				++idx;
			}
			for (int col = 0; col < width; ++col)
			{
				if (maskLambda(row, col))
				{
					*(pixelMask + idx) |= bitMask;
				}
				if (bitMask == 0x01)
				{
					bitMask = 0x80;
					++idx;
					*(pixelMask + idx) = 0x00;
				}
				else
				{
					bitMask >>= 1;
				}
			}
		}
	}
	return pixelMask;
}


class Simulation2D: public BasicWindow<Simulation2D>
{
protected:
	// Direct 2D
	ID2D1Factory* pFactory;											// Factory object
	ID2D1HwndRenderTarget* pRenderTarget;							// Render target with the application window
	ID2D1SolidColorBrush* pBrush;									// Solid color brush

	// Graphics
	HRESULT CreateGraphicsResources();
	void DiscardGraphicsResources();
	void Resize();

	// Message handlers
	void OnPaint();												// WM_Paint handler
	void OnKeyDown(UINT vkey);									// WM_KEYDOWN handler

	// Robot Simulation
	RobotSimulation robotSim;
	std::map<UnicycleRobot*, Robot2DModel*> robotModelMap;

	// Flags
	std::atomic_flag msgLoopFlag = ATOMIC_FLAG_INIT;


public:
	// Interface
	Simulation2D() = default;
	void AddRobot(UnicycleRobot* robot, Robot2DModel* model, RobotController* controller);
	void AddObstacle(RoundObstacle* obstacle);
	void MessageLoop(HINSTANCE hInstance, HINSTANCE hPrevInstance, PWSTR pCmdLine, int nCmdShow);
	void StartSimulation();
	void StopSimulation();

	PCWSTR ClassName() const { return L"2D Simulation Window Class"; }
	LRESULT HandleMessage(UINT uMsg, WPARAM wParam, LPARAM lParam) override;
};

int WinSim(HINSTANCE hInstance, HINSTANCE hPrevInstance, PWSTR pCmdLine, int nCmdShow);

int Sim2D(HINSTANCE hInstance, HINSTANCE hPrevInstance, PWSTR pCmdLine, int nCmdShow);

#endif