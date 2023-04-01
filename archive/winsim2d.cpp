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
	rotation = state[2];

	// Dimensions
	wheelWidth = (robot.collisionWidth - robot.wheelDist) / 2 * scale;
	wheelLength = wheelWidth * 3 * scale;
	bodyWidth = (robot.wheelDist - wheelWidth) * scale;
	bodyLength = (robot.collisionHeight * 0.95) * scale;
	relBodyOffset.x = relCenterRotation[0] * scale;
	relBodyOffset.y = (relCenterRotation[1] - wheelWidth) * scale;
	relLeftWheelOffset.x = -wheelLength / 2 * scale;
	relLeftWheelOffset.y = relCenterRotation[1] * scale;
	relRightWheelOffset.x = -wheelLength / 2 * scale;
	relRightWheelOffset.y = -relCenterRotation[1] * scale;

	float left, top, right, bottom;

	// Body
	left = absCenterOfRotation.x + relBodyOffset.x;
	top = absCenterOfRotation.y + relBodyOffset.y;
	right = left + bodyLength;
	bottom = top - bodyWidth;
	*body = ColorRectangle(left, top, right, bottom, rotation, relBodyOffset, D2D1::ColorF(D2D1::ColorF::Beige));

	// Left wheel
	left = absCenterOfRotation.x + relLeftWheelOffset.x;
	top = absCenterOfRotation.y + relLeftWheelOffset.y;
	right = left + wheelLength;
	bottom = top - wheelWidth;
	*leftWheel = ColorRectangle(left, top, right, bottom, rotation, relLeftWheelOffset, D2D1::ColorF(D2D1::ColorF::Black));

	// Right wheel
	left = absCenterOfRotation.x + relRightWheelOffset.x;
	top = absCenterOfRotation.y + relRightWheelOffset.y;
	right = left + wheelLength;
	bottom = top - wheelWidth;
	*rightWheel = ColorRectangle(left, top, right, bottom, rotation, relRightWheelOffset, D2D1::ColorF(D2D1::ColorF::Black));
}

Robot2DModel::Robot2DModel(UnicycleRobot& robot)
{
	// Robot
	updateParameters(robot);
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
	rotation = state[2];

	float left, top, right, bottom;

	// Body
	left = absCenterOfRotation.x + relBodyOffset.x;
	top = absCenterOfRotation.y + relBodyOffset.y;
	right = left + bodyLength;
	bottom = top - bodyWidth;
	*body = ColorRectangle(left, top, right, bottom, rotation, relBodyOffset, D2D1::ColorF(D2D1::ColorF::Beige));

	// Left wheel
	left = absCenterOfRotation.x + relLeftWheelOffset.x;
	top = absCenterOfRotation.y + relLeftWheelOffset.y;
	right = left + wheelLength;
	bottom = top - wheelWidth;
	*leftWheel = ColorRectangle(left, top, right, bottom, rotation, relLeftWheelOffset, D2D1::ColorF(D2D1::ColorF::Black));

	// Right wheel
	left = absCenterOfRotation.x + relRightWheelOffset.x;
	top = absCenterOfRotation.y + relRightWheelOffset.y;
	right = left + wheelLength;
	bottom = top - wheelWidth;
	*rightWheel = ColorRectangle(left, top, right, bottom, rotation, relRightWheelOffset, D2D1::ColorF(D2D1::ColorF::Black));

}

void Robot2DModel::Scale(UnicycleRobot& robot, float scale)
{
	this->scale = scale;
	std::vector<double> relCenterRotation = robot.relCenterRotation;
	// Dimensions
	wheelWidth = (robot.collisionWidth - robot.wheelDist) / 2 * scale;
	wheelLength = wheelWidth * 3 * scale;
	bodyWidth = (robot.wheelDist - wheelWidth) * scale;
	bodyLength = (robot.collisionHeight * 0.95) * scale;
	relBodyOffset.x = relCenterRotation[0] * scale;
	relBodyOffset.y = (relCenterRotation[1] - wheelWidth) * scale;
	relLeftWheelOffset.x = -wheelLength / 2 * scale;
	relLeftWheelOffset.y = relCenterRotation[1] * scale;
	relRightWheelOffset.x = -wheelLength / 2 * scale;
	relRightWheelOffset.y = -relCenterRotation[1] * scale;
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

		for (auto i = robotModels.begin(); i != robotModels.end(); ++i)
		{
			i->Draw(pRenderTarget, pBrush);
		}

		hr = pRenderTarget->EndDraw(); // End drawing (error signal from here)
		if (FAILED(hr) || hr == D2DERR_RECREATE_TARGET)
		{
			DiscardGraphicsResources();
		}
		EndPaint(m_hwnd, &ps);
	}
}

void Simulation2D::AddRobot(UnicycleRobot* robot, Robot2DModel* model, RobotController* controller)
{
	robotSim.AddRobot(robot, controller);
	robotModelMap.insert({ robot, model });
}

void Simulation2D::AddObstacle(RoundObstacle* obstacle)
{
	robotSim.AddObstacle(obstacle);
}

void Simulation2D::MessageLoop(HINSTANCE hInstance, HINSTANCE hPrevInstance, PWSTR pCmdLine, int nCmdShow)
{
	if (Create(L"RobotSim", WS_OVERLAPPEDWINDOW, 0, CW_USEDEFAULT, CW_USEDEFAULT,
		640, 480))
	{
		// Acceleration table
		HACCEL hAccel = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDR_ACCELERATOR1));
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

	case WM_COMMAND: // Accelerator table
		switch (LOWORD(wParam))
		{
			switch (LOWORD(wParam))
			{
			case ID_:
				SetMode(Mode::DragMode);
				break;
			case ID_SELECT_MODE:
				SetMode(Mode::SelectionMode);
				break;
			case ID_TOGGLE_MODE:
				if (mode == Mode::DrawMode) SetMode(Mode::SelectionMode);
				else SetMode(Mode::DrawMode);
				break;
			}
			return 0;
		}
		return 0;

	default:
		return DefWindowProc(m_hwnd, uMsg, wParam, lParam);
	}
}
