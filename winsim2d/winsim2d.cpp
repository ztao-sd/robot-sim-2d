#include "winsim2d.h"

template <class DerivedWindow>
BOOL BasicWindow<DerivedWindow>::Create(
	PCWSTR lpWindowName, // 
	DWORD dwStyle,
	DWORD dwExStyle,
	int x,
	int y,
	int nWidth,
	int nHeight,
	HWND hWndParent,
	HMENU hMenu
)
{
	m_wc = {};
	m_wc.lpfnWndProc = DerivedWindow::WindowProc;	// Window message handler
	m_wc.hInstance = GetModuleHandle(NULL);			// Application instance handle
	m_wc.lpszClassName = ClassName();				// Class name

	RegisterClass(&m_wc);	// Register Window class

	m_hwnd = CreateWindowEx(	// Create new instance of a window
		dwExStyle,		// Extended Window style
		ClassName(),	// Class name (optional)
		lpWindowName,	// Window text
		dwStyle,		// Window style
		x,				// X coordinate
		y,				// Y coordinate
		nWidth,			// Width
		nHeight,		// Height
		hWndParent,		// Parent Window
		hMenu,			// Menu Window
		GetModuleHandle(NULL),	// Instance handle
		this			// Additional data
	);

	return (m_hwnd ? TRUE : FALSE);
}

template <class T> void SafeRelease(T** ppT)
{
	if (*ppT)
	{
		(*ppT)->Release();
		*ppT = NULL;
	}
}

#pragma region ShapeList

template<typename Shape>
const std::unique_ptr<Shape>& ShapeList<Shape>::SelectedShape() const
{
	if (pShape) return *pShape;
	else return nullptr;
}

template<class Shape>
HRESULT ShapeList<Shape>::InsertShape(float dipX, float dipY, Shape* shape)
{
	try
	{
		if (shape)
		{
			shapes.insert(shapes.end(),
				std::unique_ptr<Shape>(shape));
		}
		else
		{
			shapes.insert(shapes.end(),
				std::unique_ptr<Shape>(new Shape(D2D1::Point2F(dipX, dipY), D2D1::ColorF(colors[colorIndex]))));
		}
		colorIndex = (colorIndex + 1) % ARRAYSIZE(colors);
		pShape = &shapes.back();
	}
	catch (std::bad_alloc)
	{
		return E_OUTOFMEMORY;
	}

	return S_OK;
}

template<class Shape>
BOOL ShapeList<Shape>::SelectShape(float dipX, float dipY)
{
	for (auto i{ shapes.rbegin() }; i != shapes.rend(); ++i) // Use reverse iterator
	{
		if ((*i)->HitTest(dipX, dipY))
		{
			// shapeIterator = (++i).base();	// Return the base iterator form reverse iterator
			pShape = &(*i);
			return TRUE;
		}
	}
	return FALSE;
}

template<typename Shape>
void ShapeList<Shape>::Draw(ID2D1RenderTarget* pRenderTarget, ID2D1SolidColorBrush* pBrush)
{
	for (auto i{ shapes.begin() }; i != shapes.end(); ++i)
	{
		(*i)->Draw(pRenderTarget, pBrush);
	}
}

#pragma endregion


void GDIBmp::Load(std::string bmpPath)
{
	Bmp::Load(bmpPath);
	bmpInfoHeader.biSize = sizeof(BITMAPINFOHEADER);
	bmpInfoHeader.biWidth = BmpInfo().width;
	bmpInfoHeader.biHeight = BmpInfo().height;
	bmpInfoHeader.biPlanes = BmpInfo().numColorPlanes;
	bmpInfoHeader.biBitCount = BmpInfo().bitsPerPixel;
	bmpInfoHeader.biCompression = BmpInfo().compression;
	bmpInfoHeader.biSizeImage = BmpInfo().imageSize;
	bmpInfoHeader.biXPelsPerMeter = BmpInfo().hResolution;
	bmpInfoHeader.biYPelsPerMeter = BmpInfo().vResolution;
	bmpInfoHeader.biClrUsed = BmpInfo().numColors;
	bmpInfoHeader.biClrImportant = BmpInfo().numImportantColors;
}

void GDIBmp::CreateMemDC(HWND hwnd)
{
	HDC hdcWindow = GetDC(hwnd); // Get context handle for a window
	hdcBmp = CreateCompatibleDC(hdcWindow);
	hBmp = CreateDIBitmap(hdcWindow, &bmpInfoHeader, CBM_INIT, Pixels().PixelPointer(),
		(BITMAPINFO*)&bmpInfoHeader, DIB_RGB_COLORS); // Create bitmap object
	SelectObject(hdcBmp, hBmp); // Select bimap in the device context
	ReleaseDC(hwnd, hdcWindow); // Release the window handle.=
}

void GDIBmp::Display(HWND hwnd)
{
	if (Pixels().PixelPointer())
	{
		// Prepare pixel data
		uint8_t* lpbitmap = Pixels().PixelPointer();

		// Retrieve device context of windows
		HDC hdcWindow = GetDC(hwnd);

		// Draw bitmap to window
		SetDIBitsToDevice(hdcWindow,
			-10, 0,
			BmpInfo().width, BmpInfo().height,
			0, 0,
			0, BmpInfo().height,
			lpbitmap,
			(BITMAPINFO*)&bmpInfoHeader,
			DIB_RGB_COLORS
		);
		ReleaseDC(hwnd, hdcWindow);
	}
}

void GDIBmp::Draw(HWND hwnd, const POINT& centroidPos, const float& orientation, uint8_t* pixelMask)
{
	// Corner position relative to centroid
	ptParallelogram[0] = POINT{
		centroidPos.x - bmpInfoHeader.biWidth / 2,
		centroidPos.y - bmpInfoHeader.biHeight / 2
	};
	ptParallelogram[1] = POINT{
		centroidPos.x + bmpInfoHeader.biWidth / 2,
		centroidPos.y - bmpInfoHeader.biHeight / 2
	};
	ptParallelogram[2] = POINT{
		centroidPos.x - bmpInfoHeader.biWidth / 2,
		centroidPos.y + bmpInfoHeader.biHeight / 2
	};
	for (POINT& pt : ptParallelogram)
	{
		const float x = pt.x;
		const float y = pt.y;
		pt.x = x * cos(orientation * S_PI / 180.0) - y * sin(orientation * S_PI / 180.0);
		pt.y = x * sin(orientation * S_PI / 180.0) + y * cos(orientation * S_PI / 180.0);
	}
	CreatePixelMask(pixelMask);
	BltTransfer(hwnd, ptParallelogram, hBmpMask);
}

void GDIBmp::CreatePixelMask(uint8_t* pixelMask)
{
	hBmpMask = CreateBitmap(bmpInfoHeader.biWidth, bmpInfoHeader.biHeight, 1, 1,
		pixelMask);
}

BOOL GDIBmp::CaptureScreen(HWND hwnd)
{
	// Get device contexts
	const HWND hDesktop = GetDesktopWindow();
	HDC hdcScreen = GetDC(NULL);					// Screen DC
	HDC hdcTemp = CreateCompatibleDC(hdcScreen);	// Temporary DC
	if (!hdcTemp)
	{
		DeleteObject(hdcTemp);
		ReleaseDC(NULL, hdcScreen);
		return FALSE;
	}

	// Get Screen size info
	RECT rectScreen;
	GetWindowRect(hDesktop, &rectScreen);

	// Create bitmap
	HBITMAP hbmpScreen = CreateCompatibleBitmap(hdcScreen, rectScreen.right - rectScreen.left,
		rectScreen.bottom - rectScreen.top);
	if (!hbmpScreen)
	{
		DeleteObject(hdcTemp);
		ReleaseDC(NULL, hdcScreen);
		return FALSE;
	}

	// Select the compatible bitmap into the compatible temp DC
	SelectObject(hdcTemp, hbmpScreen);

	// Bit block transfer into the compatible DC
	BitBlt(hdcTemp,
		0, 0,
		rectScreen.right - rectScreen.left,
		rectScreen.bottom - rectScreen.top,
		hdcScreen,
		0, 0,
		SRCCOPY);

	BITMAP bmpScreen;
	GetObject(hbmpScreen, sizeof(BITMAP), &bmpScreen);

	BITMAPFILEHEADER bmpFileHeader;
	BITMAPINFOHEADER bmpInfoHeader;

	bmpInfoHeader.biSize = sizeof(BITMAPINFOHEADER);
	bmpInfoHeader.biWidth = bmpScreen.bmWidth;
	bmpInfoHeader.biHeight = bmpScreen.bmHeight;
	bmpInfoHeader.biPlanes = 1;
	bmpInfoHeader.biBitCount = 32;
	bmpInfoHeader.biCompression = BI_RGB;
	bmpInfoHeader.biSizeImage = 0;
	bmpInfoHeader.biXPelsPerMeter = 2835;
	bmpInfoHeader.biYPelsPerMeter = 2835;
	bmpInfoHeader.biClrUsed = 0;
	bmpInfoHeader.biClrImportant = 0;

	DWORD dwBmpSize = ((bmpScreen.bmWidth * bmpInfoHeader.biBitCount + 31) / 32) * 4 * bmpScreen.bmHeight;

	HANDLE hDIB{ NULL };
	hDIB = GlobalAlloc(GHND, dwBmpSize);
	char* lpbitmap = reinterpret_cast<char*>(GlobalLock(hDIB));

	GetDIBits(hdcScreen, hbmpScreen, 0,
		(UINT)bmpScreen.bmHeight,
		lpbitmap,
		(BITMAPINFO*)&bmpInfoHeader, DIB_RGB_COLORS);

	HANDLE hFile = CreateFile(L"captureqwsx.bmp",
		GENERIC_WRITE,
		0,
		NULL,
		CREATE_ALWAYS,
		FILE_ATTRIBUTE_NORMAL, NULL);

	DWORD dwSizeofDIB = dwBmpSize + sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER);
	bmpFileHeader.bfOffBits = (DWORD)sizeof(BITMAPFILEHEADER) + (DWORD)sizeof(BITMAPINFOHEADER);
	bmpFileHeader.bfSize = dwSizeofDIB;
	bmpFileHeader.bfType = 0x4D42;

	DWORD dwBytesWritten;

	WriteFile(hFile, (LPSTR)&bmpFileHeader, sizeof(BITMAPFILEHEADER), &dwBytesWritten, NULL);
	WriteFile(hFile, (LPSTR)&bmpInfoHeader, sizeof(BITMAPINFOHEADER), &dwBytesWritten, NULL);
	WriteFile(hFile, (LPSTR)lpbitmap, dwBmpSize, &dwBytesWritten, NULL);

	// Unlock and Free the DIB from the heap.
	GlobalUnlock(hDIB);
	GlobalFree(hDIB);

	// Close the handle for the file that was created.
	CloseHandle(hFile);

	DeleteObject(hbmpScreen);
	DeleteObject(hdcTemp);
	ReleaseDC(NULL, hdcScreen);

	return TRUE;
}

char* GDIBmp::CaptureWindow(HWND hwnd)
{
	HDC hdcWindow = GetDC(hwnd);
	HDC hdcTemp = CreateCompatibleDC(hdcWindow);
	RECT rectWindow;
	GetWindowRect(hwnd, &rectWindow);
	HBITMAP hbmpWindow = CreateCompatibleBitmap(hdcWindow, rectWindow.right - rectWindow.left,
		rectWindow.bottom - rectWindow.top);
	BITMAP bmpWindow;
	SelectObject(hdcTemp, hbmpWindow);
	BitBlt(hdcTemp, 0, 0,
		rectWindow.right - rectWindow.left,
		rectWindow.bottom - rectWindow.top,
		hdcWindow, 0, 0,
		SRCCOPY);
	GetObject(hbmpWindow, sizeof(BITMAP), &bmpWindow);
	BITMAPFILEHEADER bmpFileHeader;
	BITMAPINFOHEADER bmpInfoHeader;
	bmpInfoHeader.biSize = sizeof(BITMAPINFOHEADER);
	bmpInfoHeader.biWidth = bmpWindow.bmWidth;
	bmpInfoHeader.biHeight = bmpWindow.bmHeight;
	bmpInfoHeader.biPlanes = 1;
	bmpInfoHeader.biBitCount = 24;
	bmpInfoHeader.biCompression = BI_RGB;
	bmpInfoHeader.biSizeImage = 0;
	bmpInfoHeader.biXPelsPerMeter = 2835;
	bmpInfoHeader.biYPelsPerMeter = 2835;
	bmpInfoHeader.biClrUsed = 0;
	bmpInfoHeader.biClrImportant = 0;

	DWORD dwBmpSize = ((bmpWindow.bmWidth * bmpInfoHeader.biBitCount + 31) / 32) * 4 * bmpWindow.bmHeight;
	HANDLE hDIB{ NULL };
	hDIB = GlobalAlloc(GHND, dwBmpSize);
	char* lpbitmap = reinterpret_cast<char*>(GlobalLock(hDIB));
	GetDIBits(hdcWindow, hbmpWindow, 0,
		(UINT)bmpWindow.bmHeight,
		lpbitmap,
		(BITMAPINFO*)&bmpInfoHeader, DIB_RGB_COLORS);

	return lpbitmap;
}

void GDIBmp::Release()
{
	DeleteObject(hBmp);
	DeleteObject(hdcBmp);
}

void GDIBmp::BltTransfer(HWND hwnd, const POINT* ptParallelogram, HBITMAP bMask = NULL)
{
	HDC hdcWindow = GetDC(hwnd);
	RECT rectWindow;
	GetClientRect(hwnd, &rectWindow);
	if (bMask)
	{
		PlgBlt(hdcWindow, ptParallelogram, hdcBmp, 0, 0,
			bmpInfoHeader.biWidth, bmpInfoHeader.biHeight, bMask, 0, 0);
	}
	DeleteObject(bMask);
	DeleteObject(hBmp);
	ReleaseDC(hwnd, hdcWindow);
}

uint8_t*& GDIBmp::BlackPixelMask()
{
	PixelMask([](int row, int col) {return true; }, bmpMask);
	return bmpMask;
}

uint8_t*& GDIBmp::RoundPixelMask(float radius, POINT center)
{
	const float radiusSquared = radius * radius;
	const float centerX = center.x;
	const float centerY = center.y;
	//const float centerX = Pixels().pixelMatrix.size() / 2.0f;
	//const float centerY = Pixels().pixelMatrix.front().size() / 2.0f;
	auto maskLambda = [&radiusSquared, &centerX, &centerY](int row, int col)
	{
		float r = pow((float)col - centerX, 2) + pow((float)row - centerY, 2);
		if (r < radiusSquared)
		{
			return true;
		}
		else
		{
			return false;
		}
	};
	PixelMask(maskLambda, bmpMask);
	return bmpMask;
}


int WinSim(HINSTANCE hInstance, HINSTANCE hPrevInstance, PWSTR pCmdLine, int nCmdShow)
{
	WinSim2D winSim{};

	if (!winSim.Create(L"2D Window Simulation", WS_OVERLAPPEDWINDOW, 0,
		CW_USEDEFAULT, CW_DEFAULT, 640, 480))
	{
		return 0;
	}

	ShowWindow(winSim.Window(), nCmdShow);

	// Message Loop
	MSG msg{};
	while (GetMessage(&msg, NULL, 0, 0))
	{
		TranslateMessage(&msg);
		DispatchMessage(&msg);
	}

	return 0;
}

int Sim2D(HINSTANCE hInstance, HINSTANCE hPrevInstance, PWSTR pCmdLine, int nCmdShow)
{
	UnicycleRobot* robot = new UnicycleRobot{};
	RobotController* controller = new RobotController{};
	Robot2DModel* model = new Robot2DModel{ *robot };
	Simulation2D simWin{};
	//Simulation2D* simWin = new Simulation2D{};
	simWin.AddRobot(robot, model, controller);
	simWin.StartSimulation();
	simWin.MessageLoop(hInstance, hPrevInstance, pCmdLine, nCmdShow);

	return 0;
}

int Sim2D()
{
	UnicycleRobot* robot = new UnicycleRobot{};
	RobotController* controller = new RobotController{};
	Robot2DModel* model = new Robot2DModel{ *robot };
	Simulation2D simWin{};
	//Simulation2D* simWin = new Simulation2D{};
	simWin.AddRobot(robot, model, controller);
	simWin.StartSimulation();
	simWin.MessageLoop();

	return 0;
}

HRESULT WinSim2D::CreateGraphicsResources()
{
	HRESULT hr = S_OK;
	if (pRenderTarget == NULL) // Only create resources once
	{
		RECT rc;
		GetClientRect(m_hwnd, &rc);

		D2D1_SIZE_U size = D2D1::SizeU(rc.right, rc.bottom);

		hr = pFactory->CreateHwndRenderTarget(
			D2D1::RenderTargetProperties(),					// Default options
			D2D1::HwndRenderTargetProperties(m_hwnd, size),	// Window handle & size of render target
			&pRenderTarget									// ID2D1HwndRenderTarget pointer
		);

		if (SUCCEEDED(hr))
		{
			const D2D1_COLOR_F color = D2D1::ColorF(1.0f, 1.0f, 0);		// Color
			hr = pRenderTarget->CreateSolidColorBrush(color, &pBrush);	//
		}

	}
	return hr;
}

void WinSim2D::DiscardGraphicsResources()
{
	SafeRelease(&pRenderTarget);
	SafeRelease(&pBrush);
}

void WinSim2D::Resize()
{
}

void WinSim2D::OnPaint()
{
	HRESULT hr = CreateGraphicsResources();
	if (SUCCEEDED(hr))
	{
		PAINTSTRUCT ps;
		BeginPaint(m_hwnd, &ps);
		pRenderTarget->BeginDraw(); // Start drawing
		pRenderTarget->Clear(D2D1::ColorF(D2D1::ColorF::SkyBlue));
		hr = pRenderTarget->EndDraw();
		if (FAILED(hr) || hr == D2DERR_RECREATE_TARGET)
		{
			DiscardGraphicsResources();
		}
		EndPaint(m_hwnd, &ps);
	}
	static const std::string bmpPath = R"(C:\Users\Tao\LocalRepos\image-view\image-view\obstacle.bmp)";
	bmp.Load(bmpPath);
	bmp.CreateMemDC(m_hwnd);
	bmp.Draw(m_hwnd, POINT{ 100, 100 }, 0, bmp.RoundPixelMask(39, POINT{ 40, 42 }));
}

LRESULT WinSim2D::HandleMessage(UINT uMsg, WPARAM wParam, LPARAM lParam)
{
	switch (uMsg)
	{
	case WM_CREATE:
		if (FAILED(D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &pFactory)))	// Create factory
		{
			return -1;
		}
		DPIScale::Init(m_hwnd);
		return 0;
	case WM_DESTROY:
		PostQuitMessage(0);
		return 0;
	case WM_SIZE:
		return 0;
	case WM_PAINT:
		OnPaint();
		return 0;
	default:
		return DefWindowProc(m_hwnd, uMsg, wParam, lParam);
	}
}

void Robot2DModel::updateParameters(UnicycleRobot& robot)
{
	std::vector<double> state = robot.State();
	std::vector<double> relCenterRotation = robot.relCenterRotation;

	// State
	absCenterOfRotation.x = state[0];
	absCenterOfRotation.y = state[1];
	rotation = 180 / S_PI * state[2];

	// Dimensions
	wheelWidth = (robot.collisionWidth - robot.wheelDist);
	wheelLength = wheelWidth * 3;
	bodyWidth = (robot.wheelDist - wheelWidth);
	bodyLength = (robot.collisionHeight * 0.95);
	relBodyOffset.x = relCenterRotation[0];
	relBodyOffset.y = (relCenterRotation[1] + wheelWidth);
	relLeftWheelOffset.x = -wheelLength / 2;
	relLeftWheelOffset.y = relCenterRotation[1];
	relRightWheelOffset.x = -wheelLength / 2;
	relRightWheelOffset.y = (- relCenterRotation[1] - wheelWidth);

	_Scale(scale);

	float left, top, right, bottom;

	// Body
	left = absCenterOfRotation.x + relBodyOffset.x;
	top = absCenterOfRotation.y + relBodyOffset.y;
	right = left + bodyLength;
	bottom = top + bodyWidth;
	body.reset(new ColorRectangle(left, top, right, bottom, rotation, relBodyOffset, D2D1::ColorF(D2D1::ColorF::Beige)));

	// Left wheel
	left = absCenterOfRotation.x + relLeftWheelOffset.x;
	top = absCenterOfRotation.y + relLeftWheelOffset.y;
	right = left + wheelLength;
	bottom = top + wheelWidth;
	leftWheel.reset(new ColorRectangle(left, top, right, bottom, rotation, relLeftWheelOffset, D2D1::ColorF(D2D1::ColorF::Black)));

	// Right wheel
	left = absCenterOfRotation.x + relRightWheelOffset.x;
	top = absCenterOfRotation.y + relRightWheelOffset.y;
	right = left + wheelLength;
	bottom = top + wheelWidth;
	rightWheel.reset(new ColorRectangle(left, top, right, bottom, rotation, relRightWheelOffset, D2D1::ColorF(D2D1::ColorF::Black)));
}

Robot2DModel::Robot2DModel(UnicycleRobot& robot)
{
	// Robot
	updateParameters(robot);
}

Robot2DModel::Robot2DModel(D2D1_POINT_2F cursorPos, D2D1_COLOR_F color)
{
	UnicycleRobot robot{ std::vector<double>{cursorPos.x, cursorPos.y, 0} };
	updateParameters(robot);
	body->SetColor(color);
}

void Robot2DModel::Draw(ID2D1RenderTarget* pRenderTarget, ID2D1SolidColorBrush* pBrush)
{
	if (body) body->Draw(pRenderTarget, pBrush);
	if (leftWheel) leftWheel->Draw(pRenderTarget, pBrush);
	if (rightWheel) rightWheel->Draw(pRenderTarget, pBrush);
}

void Robot2DModel::Move(FLOAT deltaX, FLOAT deltaY, FLOAT rotation)
{
	if (body) body->Move(deltaX, deltaY, rotation);
	if (leftWheel) leftWheel->Move(deltaX, deltaY, rotation);
	if (rightWheel) rightWheel->Move(deltaX, deltaY, rotation);
}

void Robot2DModel::UpdateRectangles(D2D1_POINT_2F position, FLOAT rotation)
{
	if (body) body->UpdateRectangle(position, rotation);
	if (leftWheel) leftWheel->UpdateRectangle(position, rotation);
	if (rightWheel) rightWheel->UpdateRectangle(position, rotation);
}

void Robot2DModel::UpdateState(UnicycleRobot& robot)
{
	std::vector<double> state = robot.State();
	std::vector<double> relCenterRotation = robot.relCenterRotation;

	// State
	absCenterOfRotation.x = state[0];
	absCenterOfRotation.y = state[1];
	rotation = 180 / S_PI * state[2];

	float left, top, right, bottom;

	// Body
	left = absCenterOfRotation.x + relBodyOffset.x;
	top = absCenterOfRotation.y + relBodyOffset.y;
	right = left + bodyLength;
	bottom = top + bodyWidth;
	body.reset(new ColorRectangle(left, top, right, bottom, rotation, relBodyOffset, D2D1::ColorF(D2D1::ColorF::Beige)));

	// Left wheel
	left = absCenterOfRotation.x + relLeftWheelOffset.x;
	top = absCenterOfRotation.y + relLeftWheelOffset.y;
	right = left + wheelLength;
	bottom = top + wheelWidth;
	leftWheel.reset(new ColorRectangle(left, top, right, bottom, rotation, relLeftWheelOffset, D2D1::ColorF(D2D1::ColorF::Black)));

	// Right wheel
	left = absCenterOfRotation.x + relRightWheelOffset.x;
	top = absCenterOfRotation.y + relRightWheelOffset.y;
	right = left + wheelLength;
	bottom = top + wheelWidth;
	rightWheel.reset(new ColorRectangle(left, top, right, bottom, rotation, relRightWheelOffset, D2D1::ColorF(D2D1::ColorF::Black)));

}

void Robot2DModel::Scale(UnicycleRobot& robot, float scale)
{
	this->scale = scale;
	std::vector<double> relCenterRotation = robot.relCenterRotation;
	// Dimensions
	wheelWidth = (robot.collisionWidth - robot.wheelDist);
	wheelLength = wheelWidth * 3;
	bodyWidth = (robot.wheelDist - wheelWidth);
	bodyLength = (robot.collisionHeight * 0.95);
	relBodyOffset.x = relCenterRotation[0];
	relBodyOffset.y = (relCenterRotation[1] + wheelWidth);
	relLeftWheelOffset.x = -wheelLength / 2;
	relLeftWheelOffset.y = relCenterRotation[1];
	relRightWheelOffset.x = -wheelLength / 2;
	relRightWheelOffset.y = (-relCenterRotation[1] - wheelWidth);

	_Scale(scale);
}

void Robot2DModel::_Scale(float scale)
{
	wheelWidth *= scale;
	wheelLength *= scale; 
	bodyWidth *= scale; 
	bodyLength *= scale; 
	relBodyOffset.x *= scale; 
	relBodyOffset.y *= scale; 
	relLeftWheelOffset.x *= scale; 
	relLeftWheelOffset.y *= scale; 
	relRightWheelOffset.x *= scale; 
	relRightWheelOffset.y *= scale;
}

void Robot2DModel::AddRotation(float rotation)
{
	body->Rotation() += rotation;
	leftWheel->Rotation() += rotation;
	rightWheel->Rotation() += rotation;
}

BOOL Robot2DModel::HitTest(float dipX, float dipY)
{
	if (body && leftWheel && rightWheel)
	{
		if (body->HitTest(dipX, dipY) || leftWheel->HitTest(dipX, dipY) || rightWheel->HitTest(dipX, dipY))
		{
			return TRUE;
		}
	}
	return FALSE;
}

HRESULT Simulation2D::CreateGraphicsResources()
{
	HRESULT hr = S_OK;
	if (pRenderTarget == NULL) // Only create resources once
	{
		RECT rc;
		GetClientRect(m_hwnd, &rc);
		D2D1_SIZE_U size = D2D1::SizeU(rc.right, rc.bottom);

		hr = pFactory->CreateHwndRenderTarget(
			D2D1::RenderTargetProperties(),					// Default options
			D2D1::HwndRenderTargetProperties(m_hwnd, size),	// Window handle & size of render target
			&pRenderTarget									// ID2D1HwndRenderTarget pointer
		);

		if (SUCCEEDED(hr))
		{
			const D2D1_COLOR_F color = D2D1::ColorF(1.0f, 1.0f, 0);		// Color
			hr = pRenderTarget->CreateSolidColorBrush(color, &pBrush);	//
		}
	}
	return hr;
}

void Simulation2D::DiscardGraphicsResources()
{
	SafeRelease(&pRenderTarget);
	SafeRelease(&pBrush);
}

void Simulation2D::Resize()
{
	if (pRenderTarget != NULL)
	{
		RECT rc;
		GetClientRect(m_hwnd, &rc);
		D2D1_SIZE_U size = D2D1::SizeU(rc.right, rc.bottom);
		pRenderTarget->Resize(size);
		InvalidateRect(m_hwnd, NULL, FALSE);
	}
}

void Simulation2D::OnPaint()
{
	HRESULT hr = CreateGraphicsResources();
	if (SUCCEEDED(hr))
	{
		PAINTSTRUCT ps;
		BeginPaint(m_hwnd, &ps);
		pRenderTarget->BeginDraw(); // Start drawing
		pRenderTarget->Clear(D2D1::ColorF(D2D1::ColorF::SkyBlue)); // Fill the entire render target with a solid color

		for (auto i = robotModelMap.begin(); i != robotModelMap.end(); ++i)
		{
			i->second->UpdateState(*(i->first));
			i->second->Draw(pRenderTarget, pBrush);
		}

		obsModels.Draw(pRenderTarget, pBrush);

		hr = pRenderTarget->EndDraw(); // End drawing (error signal from here)
		if (FAILED(hr) || hr == D2DERR_RECREATE_TARGET)
		{
			DiscardGraphicsResources();
		}
		EndPaint(m_hwnd, &ps);
	}
}

void Simulation2D::OnLeftButtonDown(int pixelX, int pixelY, DWORD flags)
{
	mousePos = DPIScale::PixelsToDIPs(pixelX, pixelY);

	if (mode == SimMode::Create)
	{
		POINT pt{ pixelX, pixelY };

		
		
		if (DragDetect(Window(), pt))
		{
			drawStartPos = mousePos;
			SetCapture(m_hwnd);	// Capture the mouse
			if (KEY(VK_CONTROL))
			{
				ObstacleModel* obs = new ObstacleModel{ drawStartPos, obsModels.Color(), true };
				obsModels.InsertShape(drawStartPos.x, drawStartPos.y, obs);
			}
			else if (KEY(VK_MENU))
			{
				ObstacleModel* obs = new ObstacleModel{ drawStartPos, obsModels.Color(), false };
				obsModels.InsertShape(drawStartPos.x, drawStartPos.y, obs);
			}
			else
			{
				std::vector<double> initPos{ drawStartPos.x, drawStartPos.y, 0 };
				UnicycleRobot* robot = new UnicycleRobot{ initPos };
				Robot2DModel* model = new Robot2DModel{ *robot };
				model->Scale(*robot, 0.1);
				RobotController* controller = new RobotController{};
				robotModels.InsertShape(drawStartPos.x, drawStartPos.y, model);
				AddRobot(robot, model, controller);
			}
		}
	}
	else if (mode == SimMode::Edit)
	{
		robotModels.ClearSelection();
		obsModels.ClearSelection();
		if (robotModels.SelectShape(mousePos.x, mousePos.y))
		{
			SetCapture(m_hwnd);
			UnicycleRobot& robot = *modelRobotMap[robotModels.SelectedShape().get()];
			dragObjRelPos.x = mousePos.x - robot.State()[0];
			dragObjRelPos.y = mousePos.y - robot.State()[1];
		}
		else if (obsModels.SelectShape(mousePos.x, mousePos.y))
		{
			SetCapture(m_hwnd);
			dragObjRelPos.x = mousePos.x - obsModels.SelectedShape()->Reference().x;
			dragObjRelPos.y = mousePos.y - obsModels.SelectedShape()->Reference().y;
		}
	}
	pRenderTarget->SetTransform(D2D1::Matrix3x2F::Identity()); // Rotate and translate
	InvalidateRect(m_hwnd, NULL, FALSE); // Force window to be repainted
}

void Simulation2D::OnLeftButtonUp()
{
	if ((mode == SimMode::Create))
	{
		if (robotModels.SelectedShape()) robotModels.ClearSelection();
		else if (obsModels.SelectedShape()) obsModels.ClearSelection();
		InvalidateRect(m_hwnd, NULL, FALSE);
	}
	else if (mode == SimMode::Edit)
	{
		//if (robotModels.SelectedShape()) robotModels.ClearSelection();
		//else if (obsModels.SelectedShape()) obsModels.ClearSelection();
		//InvalidateRect(m_hwnd, NULL, FALSE);
	}
	ReleaseCapture();
}

void Simulation2D::OnMouseMove(int pixelX, int pixelY, DWORD flags)
{
	const D2D1_POINT_2F dips = DPIScale::PixelsToDIPs(pixelX, pixelY);
	if (flags & MK_LBUTTON)
	{
		if (mode == SimMode::Create)
		{
			if (robotModels.SelectedShape())
			{
				UnicycleRobot& robot = *modelRobotMap[robotModels.SelectedShape().get()];
				const float size = robot.collisionHeight + robot.collisionWidth;
				float scale = (abs(dips.x - drawStartPos.x) + abs(dips.y - drawStartPos.y)) / size;
				scale = scale > 1.0f ? 1.0f : scale;
				robotModels.SelectedShape()->Scale(robot, scale);
			}
			else if (obsModels.SelectedShape())
			{
				obsModels.SelectedShape()->Scale(drawStartPos, dips);
			}

		}
		else if (mode == SimMode::Edit)
		{
			if (robotModels.SelectedShape())
			{
				UnicycleRobot& robot = *modelRobotMap[robotModels.SelectedShape().get()];
				robot.State()[0] = dips.x - dragObjRelPos.x;
				robot.State()[1] = dips.y - dragObjRelPos.y;
				robotModels.SelectedShape()->UpdateState(robot);
			}
			else if (obsModels.SelectedShape())
			{
				obsModels.SelectedShape()->Update(dips, dragObjRelPos, obsModels.SelectedShape()->Rotation());
			}
		}
		InvalidateRect(m_hwnd, NULL, FALSE);
	}
}

void Simulation2D::OnMouseWheel(int delta)
{
	if (robotModels.SelectedShape())
	{
		modelRobotMap[robotModels.SelectedShape().get()]->State()[2] += static_cast<double>(delta) * S_PI / 180 / 120.0 * 4;
	}
	else if (obsModels.SelectedShape())
	{
		obsModels.SelectedShape()->Rotation() += static_cast<float>(delta) / 120.0f * 4;
	}
	InvalidateRect(Window(), NULL, FALSE);
}

void Simulation2D::SetMode(SimMode mode)
{
	{
		LPWSTR cursor = IDC_ARROW;
		this->mode = mode;
		switch (this->mode)
		{
		case SimMode::Create:
			StopSimulation();
			cursor = IDC_CROSS;
			break;
		case SimMode::Edit:
			StopSimulation();
			cursor = IDC_HAND;
			break;
		case SimMode::Simulation:
			StartSimulation();
			break;
		}
		
		hCursor = LoadCursor(NULL, cursor);
		SetCursor(hCursor);
	}
}

void Simulation2D::AddRobot(UnicycleRobot* robot, Robot2DModel* model, RobotController* controller)
{
	robotSim.AddRobot(robot, controller);
	robotModelMap.insert({ robot, model });
	modelRobotMap.insert({ model, robot });
}

void Simulation2D::AddObstacle(RoundObstacle* obstacle)
{
	robotSim.AddObstacle(obstacle);
}

void Simulation2D::MessageLoop(HINSTANCE hInstance, HINSTANCE hPrevInstance, PWSTR pCmdLine, int nCmdShow)
{
	if (Create(L"RobotSim", WS_OVERLAPPEDWINDOW, 0, CW_USEDEFAULT, CW_USEDEFAULT,
		1280, 720))
	{
		// Acceleration table
		HACCEL hAccel = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDR_ACCELERATOR1));

		ShowWindow(this->Window(), nCmdShow);
		hCursor = LoadCursor(NULL, IDC_ARROW);
		SetCursor(hCursor);

		// Message loop
		MSG msg{};
		while (GetMessage(&msg, NULL, 0, 0))
		{
			if (!TranslateAccelerator(Window(), hAccel, &msg))
			{
				TranslateMessage(&msg);
				DispatchMessage(&msg);
			}
		}
	}
}

void Simulation2D::MessageLoop()
{
	if (Create(L"RobotSim", WS_OVERLAPPEDWINDOW, 0, CW_USEDEFAULT, CW_USEDEFAULT,
		1280, 720))
	{
		// Acceleration table
		HACCEL hAccel = LoadAccelerators(GetModuleHandle(nullptr), MAKEINTRESOURCE(IDR_ACCELERATOR1));

		ShowWindow(this->Window(), SW_SHOW);
		hCursor = LoadCursor(NULL, IDC_ARROW);
		SetCursor(hCursor);

		// Message loop
		MSG msg{};
		while (GetMessage(&msg, NULL, 0, 0))
		{
			if (!TranslateAccelerator(Window(), hAccel, &msg))
			{
				TranslateMessage(&msg);
				DispatchMessage(&msg);
			}
		}
	}
}

void Simulation2D::StartSimulation()
{
	auto f = [this]()
	{
		this->robotSim.SimStep(this->simTimer.GetInterval().count() / 1000.0);
		InvalidateRect(this->Window(), NULL, FALSE);
	};
	simTimer.Start(f, simTimer.GetInterval());
	auto g = [this]()
	{
		this->robotSim.ControlStep();
	};
	ctrlTimer.Start(g, ctrlTimer.GetInterval());
}

void Simulation2D::StopSimulation()
{
	ctrlTimer.Stop();
	simTimer.Stop();
}

void Simulation2D::CleanUp()
{
	robotSim.CleanUp();
	robotModelMap.clear();
}

LRESULT Simulation2D::HandleMessage(UINT uMsg, WPARAM wParam, LPARAM lParam)
{
	switch (uMsg)
	{
	case WM_CREATE:
		if (FAILED(D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &pFactory)))	// Create factory
		{
			return -1;
		}
		DPIScale::Init(m_hwnd);	// Initialize DPI scale class
		return 0;

	case WM_DESTROY:
		StopSimulation();
		//CleanUp();
		DiscardGraphicsResources();
		SafeRelease(&pFactory);
		PostQuitMessage(0);
		return 0;

	case WM_SIZE:
		Resize();
		return 0;

		// 2D drawing
	case WM_PAINT:
		OnPaint();
		return 0;

	case WM_LBUTTONDOWN:
		OnLeftButtonDown(GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam), (DWORD)wParam);
		return 0;
	case WM_LBUTTONUP:
		OnLeftButtonUp();
		return 0;
	case WM_MOUSEMOVE:
		OnMouseMove(GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam), (DWORD)wParam);
		return 0;
	case WM_MOUSEWHEEL:
		if (GetSystemMetrics(SM_MOUSEWHEELPRESENT) != 0)
		{
			OnMouseWheel(GET_WHEEL_DELTA_WPARAM(wParam));
		}
		return 0;

	case WM_SETCURSOR:
		//if (LOWORD(lParam) == HTCLIENT)
		//{
		//	SetCursor(hCursor);
		//	return TRUE;
		//}
		return 0;

	case WM_COMMAND: // Accelerator table
		switch (LOWORD(wParam))
		{
		case ID_CREATE_MODE:
			SetMode(SimMode::Create);
			std::cout << "Create Mode" << '\n';
			break;
		case ID_EDIT_MODE:
			SetMode(SimMode::Edit);
			std::cout << "Edit Mode" << '\n';
			break;
		case ID_SIMULATION_MODE:
			SetMode(SimMode::Simulation);
			std::cout << "Simulation Mode" << '\n';
			break;
		}
		return 0;
	default:
		return DefWindowProc(m_hwnd, uMsg, wParam, lParam);
	}
}

ObstacleModel::ObstacleModel(D2D1_POINT_2F cursorPosition, D2D1_COLOR_F color, bool circle)
{
	if (circle)
	{
		ellipse.reset(new ColorEllipse{ cursorPosition, color });
	}
	else
	{
		rectangle.reset(new ColorRectangle{ cursorPosition, color });
	}
}

ObstacleModel::ObstacleModel(D2D1_POINT_2F cursorPosition, D2D1_COLOR_F color)
{
	ellipse.reset(new ColorEllipse{ cursorPosition, color });
}

void ObstacleModel::Draw(ID2D1RenderTarget* pRenderTarget, ID2D1SolidColorBrush* pBrush)
{
	if (ellipse) ellipse->Draw(pRenderTarget, pBrush);
	else if (rectangle) rectangle->Draw(pRenderTarget, pBrush);
}

void ObstacleModel::Scale(D2D1_POINT_2F startPos, D2D1_POINT_2F curPos)
{
	if (ellipse)
	{
		auto dx = curPos.x - startPos.x;
		auto dy = curPos.y - startPos.y;
		
		
		ellipse->Shape().radiusX = abs(dx) > abs(dy) ? abs(dy / 2)  : abs(dx / 2);
		ellipse->Shape().radiusY = ellipse->Shape().radiusX;
		D2D1_POINT_2F center{ startPos.x + dx / abs(dx) * ellipse->Shape().radiusX, startPos.y + dy / abs(dy) * ellipse->Shape().radiusY };
		ellipse->Shape().point = center;
	}
	else if (rectangle)
	{
		rectangle->Shape().left = startPos.x;
		rectangle->Shape().top = startPos.y;
		rectangle->Shape().right = curPos.x;
		rectangle->Shape().bottom = curPos.y;
		rectangle->Update(D2D1_POINT_2F{ -(curPos.x - startPos.x) / 2, -(curPos.y - startPos.y) / 2 });
	}
}

void ObstacleModel::Update(D2D1_POINT_2F cursorPos, D2D1_POINT_2F relPos, float rotation)
{
	if (ellipse)
	{
		ellipse->Shape().point.x = cursorPos.x - relPos.x;
		ellipse->Shape().point.y = cursorPos.y - relPos.y;
		ellipse->Rotation() = rotation;
	}
	else if (rectangle)
	{
		rectangle->UpdateState(D2D1_POINT_2F{ cursorPos.x - relPos.x , cursorPos.y - relPos.y }, rotation);
	}
}

const D2D1_POINT_2F& ObstacleModel::Reference() const
{
	if (ellipse)
	{
		return ellipse->Shape().point;
	}
	else if (rectangle)
	{
		return D2D1_POINT_2F{ rectangle->Shape().left, rectangle->Shape().top };
	}
	return D2D1_POINT_2F();
}

float& ObstacleModel::Rotation()
{
	if (ellipse)
	{
		return ellipse->Rotation();
	}
	else if (rectangle)
	{
		return rectangle->Rotation();
	}
}

BOOL ObstacleModel::HitTest(float dipX, float dipY)
{
	if (ellipse)
	{
		if (ellipse->HitTest(dipX, dipY)) return TRUE;
	}
	else if (rectangle)
	{
		if (rectangle->HitTest(dipX, dipY)) return TRUE;
	}
	return FALSE;
}
