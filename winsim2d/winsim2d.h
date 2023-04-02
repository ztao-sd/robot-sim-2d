#ifndef WINSIM2D_H
#define WINSIM2D_H

#include "drawing_window.h"
#include "unicycle_sim.h"
#include <map>
#include <dwrite.h>
#include <iterator>



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

struct IModel
{
	virtual void Draw(ID2D1RenderTarget* pRenderTarget, ID2D1SolidColorBrush* pBrush) = 0;
	virtual BOOL HitTest(float dipX, float dipY) = 0;
	virtual void SetColor(D2D1_COLOR_F color) = 0;
};

class ObstacleModel
{
private:
	std::unique_ptr<ColorEllipse> ellipse{ nullptr };
	std::unique_ptr<ColorRectangle> rectangle{ nullptr };
	RoundObstacle roundObstacle{};
	RectObstacle rectObstacle{};
	float rotation;

public:
	ObstacleModel(D2D1_POINT_2F cursorPosition, D2D1_COLOR_F color, bool circle);
	ObstacleModel(D2D1_POINT_2F cursorPosition, D2D1_COLOR_F color);
	void Draw(ID2D1RenderTarget* pRenderTarget, ID2D1SolidColorBrush* pBrush);
	void Scale(D2D1_POINT_2F startPos, D2D1_POINT_2F curPos);
	void Update(D2D1_POINT_2F cursorPos, D2D1_POINT_2F relPos, float rotation);
	const D2D1_POINT_2F& Reference() const;
	float& Rotation();
	BOOL HitTest(float dipX, float dipY);
	BOOL CheckCollision(IModel& model);
	const std::unique_ptr<ColorEllipse>& Ellipse() const { return ellipse; }
	const std::unique_ptr<ColorRectangle>& Rectangle() const { return rectangle; }
};

class Robot2DModel: public IModel
{
private:
	FLOAT wheelLength = 50.0f;
	FLOAT wheelWidth = 10.0f;
	FLOAT bodyLength = 200.0f;
	FLOAT bodyWidth = 121.0f;
	std::unique_ptr<ColorRectangle> body{};
	std::unique_ptr<ColorRectangle> leftWheel{};
	std::unique_ptr<ColorRectangle> rightWheel{};
	Eigen::Vector2d absCenterOfRotation;
	Eigen::Vector2d relBodyOffset{ {30, bodyWidth / 2} };
	Eigen::Vector2d relLeftWheelOffset{ {30, bodyWidth / 2 + wheelWidth} };
	Eigen::Vector2d relRightWheelOffset{ {30, -(bodyWidth / 2 + wheelWidth)} };
	D2D1_RECT_F collisionRect;
	double collisionWidth;
	double collisionLength;
	float rotation;

	D2D1_COLOR_F color{ D2D1::ColorF(D2D1::ColorF::Beige) };
	void updateParameters(UnicycleWMR::Robot& robot);

public:
	FLOAT scale = 1.0f;

	// Interface
	Robot2DModel() = default;
	Robot2DModel(UnicycleWMR::Robot& robot);
	Robot2DModel(D2D1_POINT_2F cursorPos, D2D1_COLOR_F color);
	void Draw(ID2D1RenderTarget* pRenderTarget, ID2D1SolidColorBrush* pBrush);
	void Update(UnicycleWMR::Robot& robot);
	void Scale(UnicycleWMR::Robot& robot, float scale);
	void _Scale(float scale);
	BOOL HitTest(float dipX, float dipY);
	BOOL CheckCollision(Robot2DModel& model);
	BOOL CheckCollision(ObstacleModel& model);
	void SetColor(D2D1_COLOR_F color);
	const D2D1_COLOR_F& GetColor() const { return color; }
};

enum class SimMode
{
	Create,
	Edit,
	Simulation
};

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
	void OnLeftButtonDown(int pixelX, int pixelY, DWORD flags);	// WM_LBUTTONDOWN handler
	void OnLeftButtonUp();										// WN_LBUTTONUP handler
	void OnMouseMove(int pixelX, int pixelY, DWORD flags);		// WM_MOUSEMOVE handler
	void OnMouseWheel(int delta);								// WM_MOUSEWHEEL handler

	// Mouse variables
	D2D1_POINT_2F mousePos;
	D2D1_POINT_2F drawStartPos;
	D2D1_POINT_2F dragObjRelPos;

	// Utilities
	HCURSOR hCursor;
	SimMode mode;
	void SetMode(SimMode mode);

	//Timers
	Timer simTimer;
	Timer ctrlTimer;

	// Flags
	std::atomic_flag msgLoopFlag = ATOMIC_FLAG_INIT;

	// Window size
	int winWidth = 1920;
	int winHeight = 1080;

public:
	// Robot Simulation
	RobotSimulation robotSim;
	ShapeList<Robot2DModel> robotModels;
	ShapeList<ObstacleModel> obsModels;
	std::map<UnicycleWMR::Robot*, Robot2DModel*> robotModelMap;
	std::map<Robot2DModel*, UnicycleWMR::Robot*> modelRobotMap;

	// Interface
	Simulation2D(int simInterval = 10, int ctrlInterval = 20) : pFactory{ NULL }, pRenderTarget{ NULL }, pBrush{ NULL },
		robotSim{}, simTimer{ simInterval }, ctrlTimer{ ctrlInterval } {}
	void AddRobot(UnicycleWMR::Robot* robot, Robot2DModel* model);
	void AddObstacle(RoundObstacle* obstacle);
	void CheckCollision();
	void MessageLoop(HINSTANCE hInstance, HINSTANCE hPrevInstance, PWSTR pCmdLine, int nCmdShow);
	void MessageLoop();
	void StartSimulation();
	void StopSimulation();
	void CleanUp();
	//void SafeRelease();

	PCWSTR ClassName() const { return L"2D Simulation Window Class"; }
	LRESULT HandleMessage(UINT uMsg, WPARAM wParam, LPARAM lParam) override;
};

int WinSim(HINSTANCE hInstance, HINSTANCE hPrevInstance, PWSTR pCmdLine, int nCmdShow);

int Sim2D(HINSTANCE hInstance, HINSTANCE hPrevInstance, PWSTR pCmdLine, int nCmdShow);

int Sim2D();

#endif
