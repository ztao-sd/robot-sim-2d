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
	UnicycleWMR::Robot* robot = new UnicycleWMR::Robot{};
	Robot2DModel* model = new Robot2DModel{ *robot };
	Simulation2D simWin{};
	//Simulation2D* simWin = new Simulation2D{};
	simWin.AddRobot(robot, model);
	simWin.StartSimulation();
	simWin.MessageLoop(hInstance, hPrevInstance, pCmdLine, nCmdShow);

	return 0;
}

int Sim2D()
{
	//auto range = DPIScale::PixelsToDIPs(1280, 720);
	//RRT::Tree* tree = new RRT::Tree{ RRT::Vector2d{ {900, 300} }, range.x, range.y, 10 };
	//tree->ExploreN(1000);
	//TreeModel* treeModel = new TreeModel{ tree };

	//UnicycleWMR::Robot* robot = new UnicycleWMR::Robot{};
	//Robot2DModel* model = new Robot2DModel{ *robot };
	Simulation2D simWin{};
	//Simulation2D* simWin = new Simulation2D{};
	//simWin.AddTree(treeModel);
	//simWin.AddRobot(robot, model);
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

void Robot2DModel::updateParameters(UnicycleWMR::Robot& robot)
{
	auto state = robot.model.State();

	// State
	absCenterOfRotation(0) = state(0);
	absCenterOfRotation(1) = state(1);
	rotation = 180 / S_PI * state(2);

	// Dimensions
	collisionWidth = robot.model.rectCollisionLeft + robot.model.rectCollisionRight;
	collisionLength = robot.model.rectCollisionFront + robot.model.rectCollisionBack;
	wheelWidth = collisionWidth - robot.model.wheelDist;
	wheelLength = wheelWidth * 3;
	bodyWidth = robot.model.wheelDist - wheelWidth;
	bodyLength = collisionLength * 0.95;
	relBodyOffset(0) = -robot.model.centerOffsetX - robot.model.rectCollisionBack;
	relBodyOffset(1) = robot.model.centerOffsetY - robot.model.rectCollisionLeft + wheelWidth;
	relLeftWheelOffset(0) = -robot.model.centerOffsetX - wheelLength / 2;
	relLeftWheelOffset(1) = robot.model.centerOffsetY - robot.model.rectCollisionLeft;
	relRightWheelOffset(0) = -robot.model.centerOffsetX - wheelLength / 2;
	relRightWheelOffset(1) = robot.model.centerOffsetY + robot.model.rectCollisionLeft - wheelWidth;


	_Scale(scale);

	float left, top, right, bottom;

	// CollisionRect
	collisionRect.left = absCenterOfRotation(0) + relBodyOffset(0);
	collisionRect.top = absCenterOfRotation(1) + relBodyOffset(1) - wheelWidth;
	collisionRect.right = collisionRect.left + collisionLength;
	collisionRect.bottom = collisionRect.top + collisionWidth;

	// Body
	left = absCenterOfRotation(0) + relBodyOffset(0);
	top = absCenterOfRotation(1) + relBodyOffset(1);
	right = left + bodyLength;
	bottom = top + bodyWidth;
	D2D1_POINT_2F bodyOffset{ relBodyOffset(0) , relBodyOffset(1) };
	body.reset(new ColorRectangle(left, top, right, bottom, rotation, bodyOffset, color));

	// Left wheel
	left = absCenterOfRotation(0) + relLeftWheelOffset(0);
	top = absCenterOfRotation(1) + relLeftWheelOffset(1);
	right = left + wheelLength;
	bottom = top + wheelWidth;
	D2D1_POINT_2F leftOffset{ relLeftWheelOffset(0) , relLeftWheelOffset(1) };
	leftWheel.reset(new ColorRectangle(left, top, right, bottom, rotation, leftOffset, D2D1::ColorF(D2D1::ColorF::Black)));

	// Right wheel
	left = absCenterOfRotation(0) + relRightWheelOffset(0);
	top = absCenterOfRotation(1) + relRightWheelOffset(1);
	right = left + wheelLength;
	bottom = top + wheelWidth;
	D2D1_POINT_2F reightOffset{ relRightWheelOffset(0) , relRightWheelOffset(1) };
	rightWheel.reset(new ColorRectangle(left, top, right, bottom, rotation, reightOffset, D2D1::ColorF(D2D1::ColorF::Black)));
}

Robot2DModel::Robot2DModel(UnicycleWMR::Robot& robot)
{
	// Robot
	updateParameters(robot);
}

Robot2DModel::Robot2DModel(D2D1_POINT_2F cursorPos, D2D1_COLOR_F color)
{
	UnicycleWMR::Robot robot{ Eigen::Vector3d{cursorPos.x, cursorPos.y, 0} };
	updateParameters(robot);
	body->SetColor(color);
}

void Robot2DModel::Draw(ID2D1RenderTarget* pRenderTarget, ID2D1SolidColorBrush* pBrush)
{
	pRenderTarget->SetTransform(D2D1::Matrix3x2F::Rotation(rotation, D2D1_POINT_2F{ (float)absCenterOfRotation(0),(float)absCenterOfRotation(1) }));
	pBrush->SetColor(color);
	//pRenderTarget->FillRectangle(collisionRect, pBrush);
	pBrush->SetColor(D2D1::ColorF(D2D1::ColorF::BlueViolet));
	pRenderTarget->DrawRectangle(collisionRect, pBrush, 1.0f);
	pRenderTarget->SetTransform(D2D1::Matrix3x2F::Identity());
	if (body) body->Draw(pRenderTarget, pBrush);
	if (leftWheel) leftWheel->Draw(pRenderTarget, pBrush);
	if (rightWheel) rightWheel->Draw(pRenderTarget, pBrush);
}

void Robot2DModel::Update(UnicycleWMR::Robot& robot)
{
	auto state = robot.model.State();

	// State
	absCenterOfRotation(0) = state(0);
	absCenterOfRotation(1) = state(1);
	rotation = 180 / S_PI * state(2);

	float left, top, right, bottom;

	// CollisionRect
	collisionRect.left = absCenterOfRotation(0) + relBodyOffset(0);
	collisionRect.top = absCenterOfRotation(1) + relBodyOffset(1) - wheelWidth;
	collisionRect.right = collisionRect.left + collisionLength;
	collisionRect.bottom = collisionRect.top + collisionWidth;

	// Body
	left = absCenterOfRotation(0) + relBodyOffset(0);
	top = absCenterOfRotation(1) + relBodyOffset(1);
	right = left + bodyLength;
	bottom = top + bodyWidth;
	D2D1_POINT_2F bodyOffset{ relBodyOffset(0) , relBodyOffset(1) };
	body.reset(new ColorRectangle(left, top, right, bottom, rotation, bodyOffset, color));

	// Left wheel
	left = absCenterOfRotation(0) + relLeftWheelOffset(0);
	top = absCenterOfRotation(1) + relLeftWheelOffset(1);
	right = left + wheelLength;
	bottom = top + wheelWidth;
	D2D1_POINT_2F leftOffset{ relLeftWheelOffset(0) , relLeftWheelOffset(1) };
	leftWheel.reset(new ColorRectangle(left, top, right, bottom, rotation, leftOffset, D2D1::ColorF(D2D1::ColorF::Black)));

	// Right wheel
	left = absCenterOfRotation(0) + relRightWheelOffset(0);
	top = absCenterOfRotation(1) + relRightWheelOffset(1);
	right = left + wheelLength;
	bottom = top + wheelWidth;
	D2D1_POINT_2F reightOffset{ relRightWheelOffset(0) , relRightWheelOffset(1) };
	rightWheel.reset(new ColorRectangle(left, top, right, bottom, rotation, reightOffset, D2D1::ColorF(D2D1::ColorF::Black)));

}

void Robot2DModel::Scale(UnicycleWMR::Robot& robot, float scale)
{
	this->scale = scale;
	// Dimensions
	collisionWidth = robot.model.rectCollisionLeft + robot.model.rectCollisionRight;
	collisionLength = robot.model.rectCollisionFront + robot.model.rectCollisionBack;
	wheelWidth = collisionWidth - robot.model.wheelDist;
	wheelLength = wheelWidth * 3;
	bodyWidth = robot.model.wheelDist - wheelWidth;
	bodyLength = collisionLength * 0.95;
	relBodyOffset(0) = -robot.model.centerOffsetX - robot.model.rectCollisionBack;
	relBodyOffset(1) = robot.model.centerOffsetY - robot.model.rectCollisionLeft + wheelWidth;
	relLeftWheelOffset(0) = -robot.model.centerOffsetX - wheelLength / 2;
	relLeftWheelOffset(1) = robot.model.centerOffsetY - robot.model.rectCollisionLeft;
	relRightWheelOffset(0) = -robot.model.centerOffsetX - wheelLength / 2;
	relRightWheelOffset(1) = robot.model.centerOffsetY + robot.model.rectCollisionLeft - wheelWidth;

	_Scale(scale);
}

void Robot2DModel::_Scale(float scale)
{
	collisionWidth *= scale;
	collisionLength *= scale;
	wheelWidth *= scale;
	wheelLength *= scale; 
	bodyWidth *= scale; 
	bodyLength *= scale; 
	relBodyOffset(0) *= scale; 
	relBodyOffset(1) *= scale; 
	relLeftWheelOffset(0) *= scale; 
	relLeftWheelOffset(1) *= scale; 
	relRightWheelOffset(0) *= scale; 
	relRightWheelOffset(1) *= scale;
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

void Rotate2DVector(Eigen::Vector2d& vector, double rotation)
{
	Eigen::Matrix2d rotationMatrix;
	rotationMatrix << cos(rotation * S_PI / 180.0), -sin(rotation * S_PI / 180.0),
		sin(rotation * S_PI / 180.0), cos(rotation * S_PI / 180.0);
	vector = rotationMatrix * vector;
}

BOOL Robot2DModel::CheckCollision(Robot2DModel& model)
{
	// Source model
	Eigen::Vector2d centerSource = absCenterOfRotation;

	// Target model
	Eigen::Vector2d centerTarget = model.absCenterOfRotation;
	Eigen::Vector2d topLeftTarget{ {model.collisionRect.left, model.collisionRect.top}};
	Eigen::Vector2d topRightTarget{ {model.collisionRect.right, model.collisionRect.top} };
	Eigen::Vector2d bottomLeftTarget{ {model.collisionRect.left, model.collisionRect.bottom} };
	Eigen::Vector2d bottomRightTarget{ {model.collisionRect.right, model.collisionRect.bottom} };
	topLeftTarget -= centerTarget;
	topRightTarget -= centerTarget;
	bottomLeftTarget -= centerTarget;
	bottomRightTarget -= centerTarget;
	Rotate2DVector(topLeftTarget, model.rotation);
	Rotate2DVector(topRightTarget, model.rotation);
	Rotate2DVector(bottomLeftTarget, model.rotation);
	Rotate2DVector(bottomRightTarget, model.rotation);
	topLeftTarget += centerTarget;
	topRightTarget += centerTarget;
	bottomLeftTarget += centerTarget;
	bottomRightTarget += centerTarget;

	if (HitTest(topLeftTarget(0), topLeftTarget(1)) || HitTest(topRightTarget(0), topRightTarget(1))
		|| HitTest(bottomLeftTarget(0), bottomLeftTarget(1)) || HitTest(bottomRightTarget(0), bottomRightTarget(1)))
	{
		return TRUE;
	}
	return FALSE;
}

double DistanceToLine2D(Eigen::Vector2d point1, Eigen::Vector2d point2, Eigen::Vector2d p0)
{
	double length = std::sqrt(std::pow(point2(0) - point1(0), 2.0) + std::pow(point2(1) - point1(1), 2.0));
	return abs((point2(0)-point1(0))*(point1(1)-p0(1)) - (point1(0)-p0(0))*(point2(1)-point1(1))) / length;
}

Eigen::Vector2d VectorToLine2D(Eigen::Vector2d point1, Eigen::Vector2d point2, Eigen::Vector2d p0)
{
	Eigen::Vector2d n = (point2 - point1).normalized();
	return (point1 - p0) - ((point1 - p0).dot(n)) * n;
}

BOOL CheckBetween(Eigen::Vector2d point1, Eigen::Vector2d point2, Eigen::Vector2d p0)
{
	return ((point1(0) < point2(0)) ? (p0(0) >= point1(0) && p0(0) <= point2(0)) : (p0(0) >= point2(0) && p0(0) <= point1(0))) &&
		((point1(1) < point2(1)) ? (p0(1) >= point1(1) && p0(1) <= point2(1)) : (p0(1) >= point2(1) && p0(1) <= point1(1)));
}

BOOL Robot2DModel::CheckCollision(ObstacleModel& model)
{
	Eigen::Vector2d centerTarget = absCenterOfRotation;
	Eigen::Vector2d topLeftTarget{ {collisionRect.left, collisionRect.top} };
	Eigen::Vector2d topRightTarget{ {collisionRect.right, collisionRect.top} };
	Eigen::Vector2d bottomLeftTarget{ {collisionRect.left, collisionRect.bottom} };
	Eigen::Vector2d bottomRightTarget{ {collisionRect.right, collisionRect.bottom} };
	topLeftTarget -= centerTarget;
	topRightTarget -= centerTarget;
	bottomLeftTarget -= centerTarget;
	bottomRightTarget -= centerTarget;
	Rotate2DVector(topLeftTarget, rotation);
	Rotate2DVector(topRightTarget, rotation);
	Rotate2DVector(bottomLeftTarget, rotation);
	Rotate2DVector(bottomRightTarget, rotation);
	topLeftTarget += centerTarget;
	topRightTarget += centerTarget;
	bottomLeftTarget += centerTarget;
	bottomRightTarget += centerTarget;
	if (model.Ellipse())
	{
		Eigen::Vector2d center{ { model.Ellipse()->Shape().point.x, model.Ellipse()->Shape().point.y} };
		Eigen::Vector2d vector;

		if ((topLeftTarget - center).norm() <= model.Ellipse()->Shape().radiusX)
		{
			return TRUE;
		}
		if ((topRightTarget - center).norm() <= model.Ellipse()->Shape().radiusX)
		{
			return TRUE;
		}
		if ((bottomLeftTarget - center).norm() <= model.Ellipse()->Shape().radiusX)
		{
			return TRUE;
		}
		if ((bottomRightTarget - center).norm() <= model.Ellipse()->Shape().radiusX)
		{
			return TRUE;
		}

		vector = VectorToLine2D(topLeftTarget, topRightTarget, center);
		if (vector.norm() <= model.Ellipse()->Shape().radiusX && CheckBetween(topLeftTarget, topRightTarget, center + vector))
			return TRUE;
		vector = VectorToLine2D(topRightTarget, bottomRightTarget, center);
		if (vector.norm() <= model.Ellipse()->Shape().radiusX && CheckBetween(topRightTarget, bottomRightTarget, center + vector))
			return TRUE;
		vector = VectorToLine2D(bottomRightTarget, bottomLeftTarget, center);
		if (vector.norm() <= model.Ellipse()->Shape().radiusX && CheckBetween(bottomRightTarget, bottomLeftTarget, center + vector))
			return TRUE;
		vector = VectorToLine2D(bottomLeftTarget, topLeftTarget, center);
		if (vector.norm() <= model.Ellipse()->Shape().radiusX && CheckBetween(bottomLeftTarget, topLeftTarget, center + vector))
			return TRUE;

		//if (DistanceToLine2D(topLeftTarget, topRightTarget, center) <= model.Ellipse()->Shape().radiusX ||
		//	DistanceToLine2D(topRightTarget, bottomRightTarget, center) <= model.Ellipse()->Shape().radiusX ||
		//	DistanceToLine2D(bottomRightTarget, bottomLeftTarget, center) <= model.Ellipse()->Shape().radiusX ||
		//	DistanceToLine2D(bottomLeftTarget, topLeftTarget, center) <= model.Ellipse()->Shape().radiusX)
		//{
		//	return TRUE;
		//}
	}
	else if (model.Rectangle())
	{

	}
	return FALSE;
}

void Robot2DModel::SetColor(D2D1_COLOR_F color)
{
	if (body)
	{
		body->SetColor(color);
		this->color = color;
	}
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

		CheckCollision();

		for (auto i = robotModelMap.begin(); i != robotModelMap.end(); ++i)
		{
			i->second->Update(*(i->first));
			i->second->Draw(pRenderTarget, pBrush);
		}
		
		obsModels.Draw(pRenderTarget, pBrush);
		treeModels.Draw(pRenderTarget, pBrush);
		curveModels.Draw(pRenderTarget, pBrush);

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
				Eigen::Vector3d initPos{ drawStartPos.x, drawStartPos.y, 0 };
				UnicycleWMR::Robot* robot = new UnicycleWMR::Robot{ initPos };
				robot->controller = new UnicycleWMR::Controller();
				robot->controller->robot = &(robot->model);
				Robot2DModel* model = new Robot2DModel{ *robot };
				model->Scale(*robot, 0.1);
				robotModels.InsertShape(drawStartPos.x, drawStartPos.y, model);
				AddRobot(robot, model);
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
			UnicycleWMR::Robot& robot = *modelRobotMap[robotModels.SelectedShape().get()];
			dragObjRelPos.x = mousePos.x - robot.model.State()(0);
			dragObjRelPos.y = mousePos.y - robot.model.State()(1);
		}
		else if (obsModels.SelectShape(mousePos.x, mousePos.y))
		{
			SetCapture(m_hwnd);
			dragObjRelPos.x = mousePos.x - obsModels.SelectedShape()->Reference().x;
			dragObjRelPos.y = mousePos.y - obsModels.SelectedShape()->Reference().y;
		}
	}
	else if (mode == SimMode::Simulation)
	{
		if (robotModels.SelectedShape())
		{
			// Stop simulation
			StopControl();
			controlFlag = false;

			Vector3d home;
			Vector3d target;
			UnicycleWMR::Robot* robot = modelRobotMap[robotModels.SelectedShape().get()];
			robot->controller->Reset(robot->model); // Reset control
			home << robot->model.State()(0), robot->model.State()(1), robot->model.State()(2);
			target << mousePos.x, mousePos.y, robot->model.State()(2);
			if (robot->pathPlanner->PlanPath(home, target, &obsModels))
			{
				if (robotTreeMap.find(robot) != robotTreeMap.end())
				{
					robotTreeMap[robot]->Reset(robot->pathPlanner->tree.get());
				}
				else {
					TreeModel* treeModel = new TreeModel{ robot->pathPlanner->tree.get() };
					AddTree(treeModel);
					robotTreeMap.insert({ robot, treeModel });
				}
				if (robotCurveMap.find(robot) != robotCurveMap.end())
				{
					robotCurveMap[robot]->Reset(robot->pathPlanner->pathCurve.get());
				}
				else {
					CurveModel* curveModel = new CurveModel{ robot->pathPlanner->pathCurve.get() };
					AddCurve(curveModel);
					robotCurveMap.insert({ robot, curveModel });
				}
			}
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
				UnicycleWMR::Robot& robot = *modelRobotMap[robotModels.SelectedShape().get()];
				const float size = robot.model.rectCollisionLeft + robot.model.rectCollisionFront 
					+ robot.model.rectCollisionFront + robot.model.rectCollisionBack;
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
				UnicycleWMR::Robot& robot = *modelRobotMap[robotModels.SelectedShape().get()];
				robot.model.State()(0) = dips.x - dragObjRelPos.x;
				robot.model.State()(1) = dips.y - dragObjRelPos.y;
				robotModels.SelectedShape()->Update(robot);
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
		modelRobotMap[robotModels.SelectedShape().get()]->model.State()(2) += static_cast<double>(delta) * S_PI / 180 / 120.0 * 4;
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

void Simulation2D::AddRobot(UnicycleWMR::Robot* robot, Robot2DModel* model)
{
	robotSim.AddRobot(robot);
	robotModelMap.insert({ robot, model });
	modelRobotMap.insert({ model, robot });
}

void Simulation2D::AddObstacle(RoundObstacle* obstacle)
{
	robotSim.AddObstacle(obstacle);
}

void Simulation2D::AddTree(TreeModel* tree)
{
	treeModels.InsertShape(0, 0, tree);
}

void Simulation2D::AddCurve(CurveModel* curve)
{
	curveModels.InsertShape(0, 0, curve);
}

void Simulation2D::CheckCollision()
{
	for (auto i{ robotModels.List().begin() }; i != robotModels.List().end(); ++i)
	{
		(*i)->SetColor(D2D1::ColorF(D2D1::ColorF::Beige));
	}
	for (auto i{ robotModels.List().begin() }; i != robotModels.List().end(); ++i)
	{
		for (auto j{ std::next(i,1) }; j != robotModels.List().end(); ++j)
		{
			if ((*i)->CheckCollision(*(*j)) || (*j)->CheckCollision(*(*i)))
			{
				(*i)->SetColor(D2D1::ColorF(D2D1::ColorF::Red));
				(*j)->SetColor(D2D1::ColorF(D2D1::ColorF::Red));
			}
		}

		for (auto k{ obsModels.List().begin() }; k != obsModels.List().end(); ++k)
		{
			if ((*i)->CheckCollision(*(*k)))
			{
				(*i)->SetColor(D2D1::ColorF(D2D1::ColorF::Red));
			}
		}
	}
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
	
}

void Simulation2D::StopSimulation()
{
	ctrlTimer.Stop();
	simTimer.Stop();
}

void Simulation2D::StartControl()
{
	auto robot = modelRobotMap[this->robotModels.SelectedShape().get()];
	auto& model = robot->model;
	auto& curve = *(robot->pathPlanner->pathCurve);
	robot->controller->Reset(model);
	auto g = [this, robot, &model, &curve]()
	{
		robot->controller->UnifiedControl(model, curve);
		//this->robotSim.ControlStep();
	};
	ctrlTimer.Start(g, ctrlTimer.GetInterval());
}

void Simulation2D::StopControl()
{
	auto robot = modelRobotMap[this->robotModels.SelectedShape().get()];
	robot->controller->Reset(robot->model);
	ctrlTimer.Stop();
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
		case ID_CONTROL:
			if (!controlFlag)
			{
				StartControl();
				controlFlag = true;
			}
			else
			{
				StopControl();
				controlFlag = false;
			}
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

BOOL ObstacleModel::CheckCollision(IModel& model)
{
	return 0;
}

TreeModel::TreeModel(RRT::Tree* tree) : tree{ tree }
{
}

void TreeModel::Reset(RRT::Tree* tree)
{
	this->tree = tree;
}

void TreeModel::Draw(ID2D1RenderTarget* pRenderTarget, ID2D1SolidColorBrush* pBrush)
{
	pBrush->SetColor(color);
	RRT::Tree::EdgeIterator ei, ei_end;
	for (tie(ei, ei_end) = edges(tree->Graph()); ei != ei_end; ++ei)
	{
		auto src = source(*ei, tree->Graph());
		auto trg = target(*ei, tree->Graph());
		D2D1_POINT_2F point0{ tree->Graph()[src].state(0), tree->Graph()[src].state(1) };
		D2D1_POINT_2F point1{ tree->Graph()[trg].state(0), tree->Graph()[trg].state(1) };
		pRenderTarget->DrawLine(point0, point1, pBrush, 1.0f);
	}
	if (!tree->ShortestPath().empty())
	{
		pBrush->SetColor(D2D1::ColorF(D2D1::ColorF::PaleVioletRed));
		for (int i = 0; i < tree->ShortestPath().size() - 1; ++i)
		{
			D2D1_POINT_2F point0{ tree->ShortestPath()[i](0), tree->ShortestPath()[i](1) };
			D2D1_POINT_2F point1{ tree->ShortestPath()[i + 1](0), tree->ShortestPath()[i + 1](1) };
			pRenderTarget->DrawLine(point0, point1, pBrush, 1.0f);
		}
	}
}

void TreeModel::SetColor(D2D1_COLOR_F color)
{
	this->color = color;
}

void CurveModel::Draw(ID2D1RenderTarget* pRenderTarget, ID2D1SolidColorBrush* pBrush)
{
	pBrush->SetColor(color);
	double time = 0.0;
	double timeStep = (curve->endTime - curve->startTime) / 100;
	for (int i = 0; i < 100; ++i)
	{
		auto point0 = (*curve)(time);
		time += timeStep;
		auto point1 = (*curve)(time);
		//time += timeStep;
		D2D1_POINT_2F p0{ point0(0), point0(1)};
		D2D1_POINT_2F p1{ point1(0), point1(1)};
		pRenderTarget->DrawLine(p0, p1, pBrush, 1.0f);
	}
}
