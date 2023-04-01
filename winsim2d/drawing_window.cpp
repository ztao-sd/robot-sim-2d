#include "drawing_window.h"

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

template <class T> 
void SafeRelease(T** ppT)
{
	if (*ppT)
	{
		(*ppT)->Release();
		*ppT = NULL;
	}
}

// ----------------------------------------------------------------------
// Drawing Window
// ----------------------------------------------------------------------

LRESULT DrawingWindow::HandleMessage(UINT uMsg, WPARAM wParam, LPARAM lParam)
{
	switch (uMsg)
	{
	case WM_CREATE:
		if (FAILED(D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &pFactory)))	// Create factory
		{
			return -1; 
		}
		DPIScale::Init(m_hwnd);	// Initialize DPI scale class
		SetMode(Mode::DrawMode);
		
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

	// Mouse input
	// lParam containes the x and y coordinates of the mouse pointer.
	case WM_LBUTTONDOWN:
		OnLeftButtonDown(GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam), (DWORD)wParam);
		return 0;
	case WM_LBUTTONUP:
		OnLeftButtonUp();
		return 0;
	case WM_RBUTTONDOWN:
		OnRightButtonDown(GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam), (DWORD)wParam);
		return 0;
	case WM_RBUTTONUP:
		OnRightButtonUp();
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

	// Set cursor
	case WM_SETCURSOR:
		if (LOWORD(lParam) == HTCLIENT)
		{
			SetCursor(hCursor);
			return TRUE;
		}
		break;

	// Keyboard input
	case WM_KEYDOWN:
	case WM_COMMAND: // Accelerator table
		switch (LOWORD(wParam))
		{
		case ID_DRAW_MODE:
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

	default:
		return DefWindowProc(m_hwnd, uMsg, wParam, lParam);
	}
}

void DrawingWindow::SetMode(Mode mode)
{
	{
		LPWSTR cursor = IDC_ARROW;
		this->mode = mode;
		switch (this->mode)
		{
		case Mode::DrawMode:
			cursor = IDC_CROSS;
			break;
		case Mode::SelectionMode:
			cursor = IDC_HAND;
			break;
		case Mode::DragMode:
			cursor = IDC_SIZEALL;
			break;
		}

		hCursor = LoadCursor(NULL, cursor);
		SetCursor(hCursor);
	}
}

HRESULT DrawingWindow::CreateGraphicsResources()
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

			if (SUCCEEDED(hr))
			{
				//CalculateLayout();
			}
		}

	}
	return hr;
}

void DrawingWindow::DiscardGraphicsResources()
{
	SafeRelease(&pRenderTarget);
	SafeRelease(&pBrush);
}

void DrawingWindow::Resize()
{
	if (pRenderTarget != NULL)
	{
		RECT rc;
		GetClientRect(m_hwnd, &rc);

		D2D1_SIZE_U size = D2D1::SizeU(rc.right, rc.bottom);

		pRenderTarget->Resize(size);
		//CalculateLayout();
		InvalidateRect(m_hwnd, NULL, FALSE);
	}
}

void DrawingWindow::OnPaint()
{
	static const std::string bmpPath = R"(C:\Users\Tao\LocalRepos\image-view\image-view\obstacle.bmp)";
	HRESULT hr = CreateGraphicsResources();
	if (SUCCEEDED(hr))
	{
		PAINTSTRUCT ps;
		BeginPaint(m_hwnd, &ps);

		//D2D1_SIZE_F size = pRenderTarget->GetSize();
		//const float x = size.width / 2;
		//const float y = size.height / 2;
		//const D2D1::Matrix3x2F rot = D2D1::Matrix3x2F::Rotation(20, D2D1::Point2F(x, y));
		//const D2D1::Matrix3x2F trans = D2D1::Matrix3x2F::Translation(100, 50);

		pRenderTarget->BeginDraw(); // Start drawing

		pRenderTarget->Clear(D2D1::ColorF(D2D1::ColorF::SkyBlue)); // Fill the entire render target with a solid color
		//pRenderTarget->FillEllipse(ellipse, pBrush); // Draw a filled ellipse

		ellipses.Draw(pRenderTarget, pBrush);

		rectangles.Draw(pRenderTarget, pBrush);

		hr = pRenderTarget->EndDraw(); // End drawing (error signal from here)
		if (FAILED(hr) || hr == D2DERR_RECREATE_TARGET)
		{
			DiscardGraphicsResources();
		}
		//bmp.Load(bmpPath);
		//bmp.CreateMemDC(m_hwnd);
		//bmp.Draw(m_hwnd, POINT{ 200, 200 }, 0);

		EndPaint(m_hwnd, &ps);
	}
}

void DrawingWindow::OnLeftButtonDown(int pixelX, int pixelY, DWORD flags)
{

	ptMouse = DPIScale::PixelsToDIPs(pixelX, pixelY);

	if (mode == Mode::DrawMode)
	{
		POINT pt{ pixelX, pixelY };
		if (DragDetect(m_hwnd, pt))
		{
			drawStartPos = ptMouse;
			SetCapture(m_hwnd);	// Capture the mouse
			ellipses.InsertShape(drawStartPos.x, drawStartPos.y);
		}
	}
	else
	{
		ellipses.ClearSelection();
		if (ellipses.SelectShape(ptMouse.x, ptMouse.y))
		{
			SetCapture(m_hwnd);
			dragObjRelPos = ellipses.SelectedShape()->Shape().point;
			dragObjRelPos.x -= ptMouse.x;
			dragObjRelPos.y -= ptMouse.y;
			SetMode(Mode::DragMode);
		}
	}
	pRenderTarget->SetTransform(D2D1::Matrix3x2F::Identity()); // Rotate and translate

	InvalidateRect(m_hwnd, NULL, FALSE); // Force window to be repainted
}

void DrawingWindow::OnMouseMove(int pixelX, int pixelY, DWORD flags)
{
	const D2D1_POINT_2F dips = DPIScale::PixelsToDIPs(pixelX, pixelY);
	if (flags & MK_LBUTTON)
	{
		if (mode == Mode::DrawMode)
		{
			const float width = (dips.x - drawStartPos.x) / 2;
			const float height = (dips.y - drawStartPos.y) / 2;
			const float x1 = drawStartPos.x + width;
			const float y1 = drawStartPos.y + height;
			//ellipse = D2D1::Ellipse(D2D1::Point2F(x1, y1), width, height);

			if (ellipses.SelectedShape())
			{
				ellipses.SelectedShape()->Shape() = D2D1::Ellipse(D2D1::Point2F(x1, y1), width, height);
			}

		}
		else if (mode == Mode::DragMode)
		{
			if (ellipses.SelectedShape())
			{
				ellipses.SelectedShape()->Shape().point.x = dips.x + dragObjRelPos.x;
				ellipses.SelectedShape()->Shape().point.y = dips.y + dragObjRelPos.y;
			}
		}
		InvalidateRect(m_hwnd, NULL, FALSE);
	}
	else if (flags & MK_RBUTTON)
	{
		if (mode == Mode::DrawMode)
		{
			const float left = drawStartPos.x;
			const float top = drawStartPos.y;
			const float right = dips.x;
			const float bottom = dips.y;

			if (rectangles.SelectedShape())
			{

				rectangles.SelectedShape()->Shape().right = dips.x;
				rectangles.SelectedShape()->Shape().bottom = dips.y;
			}
		}
		else if (mode == Mode::DragMode)
		{
			if (rectangles.SelectedShape())
			{
				rectangles.SelectedShape()->Move(
					dips.x - dragObjRelPos.x,
					dips.y - dragObjRelPos.y,
					0
				);
				dragObjRelPos = dips;
			}
		}
		InvalidateRect(m_hwnd, NULL, FALSE);
	}
}

void DrawingWindow::OnLeftButtonUp()
{
	if ((mode == Mode::DrawMode))
	{
		if (ellipses.SelectedShape()) ellipses.ClearSelection();
		InvalidateRect(m_hwnd, NULL, FALSE);
	}
	else if (mode == Mode::DragMode)
	{
		SetMode(Mode::SelectionMode);
	}
	ReleaseCapture();
}

void DrawingWindow::OnRightButtonDown(int pixelX, int pixelY, DWORD flags)
{
	ptMouse = DPIScale::PixelsToDIPs(pixelX, pixelY);

	if (mode == Mode::DrawMode)
	{
		POINT pt{ pixelX, pixelY };
		drawStartPos = ptMouse;
		SetCapture(m_hwnd);	// Capture the mouse
		rectangles.InsertShape(drawStartPos.x, drawStartPos.y);
	}
	else
	{
		rectangles.ClearSelection();
		if (rectangles.SelectShape(ptMouse.x, ptMouse.y))
		{
			SetCapture(m_hwnd);
			dragObjRelPos = ptMouse;
			SetMode(Mode::DragMode);
		}
	}
	pRenderTarget->SetTransform(D2D1::Matrix3x2F::Identity()); // Rotate and translate
	InvalidateRect(m_hwnd, NULL, FALSE); // Force window to be repainted
}

void DrawingWindow::OnRightButtonUp()
{
	if ((mode == Mode::DrawMode))
	{
		if (rectangles.SelectedShape()) 
		{
			D2D1_POINT_2F relCenter{
				(rectangles.SelectedShape()->Shape().left - rectangles.SelectedShape()->Shape().right) / 2,
				(rectangles.SelectedShape()->Shape().top - rectangles.SelectedShape()->Shape().bottom) / 2
			};
			rectangles.SelectedShape()->Update(relCenter);
			rectangles.ClearSelection();
		}
		InvalidateRect(m_hwnd, NULL, FALSE);
	}
	else if (mode == Mode::DragMode)
	{
		SetMode(Mode::SelectionMode);
	}
	ReleaseCapture();
}

void DrawingWindow::OnMouseWheel(int delta)
{
	if (ellipses.SelectedShape())
	{
		ellipses.SelectedShape()->Rotation() += static_cast<float>(delta) / 120.0f * 4; // Rotation angle in degrees
	}

	if (rectangles.SelectedShape())
	{
		rectangles.SelectedShape()->Rotation() += static_cast<float>(delta) / 120.0f * 4;
	}

	//const D2D1::Matrix3x2F rot = D2D1::Matrix3x2F::Rotation(angle, ellipse.point);
	//pRenderTarget->SetTransform(rot); // Rotate and translate


	InvalidateRect(m_hwnd, NULL, FALSE);
}

void DrawingWindow::OnKeyDown(UINT vkey)
{
	switch (vkey)
	{
	case VK_BACK:
		break;
	case VK_DELETE:
		break;
	case VK_LEFT:
		break;
	case VK_RIGHT:
		break;
	case VK_UP:
		break;
	case VK_DOWN:
		break;
	default:
		return;
	}
}

void DrawingWindow::CaptureBitmap()
{
	/*
	HDC: device context handle
	HBITMAP: bitmap handle
	BITMAP: bitmap info struct
	DWORD: unsigned long int
	HANDLE: void pointer
	RECT: rectangle info struct
	*/

	HDC hdcScreen;				// Device context handle
	HDC hdcWindow;
	HDC hdcMemDC = NULL;
	HBITMAP hbmScreen = NULL;	// Bitmap handle
	BITMAP bmpScreen;
	DWORD dwBytesWritten = 0;
	DWORD dwSizeofDIB = 0;
	HANDLE hFile = NULL;
	char* lpbitmap = NULL;
	HANDLE hDIB = NULL;
	DWORD dwBmpSize = 0;

	hdcScreen = GetDC(NULL);
	hdcWindow = GetDC(m_hwnd);

	hdcMemDC = CreateCompatibleDC(hdcWindow);

	if (!hdcMemDC)
	{
		MessageBox(m_hwnd, L"CreateCompatibleDC has failed", L"Failed", MB_OK);
		goto done;
	}

	RECT rcClient;
	GetClientRect(m_hwnd, &rcClient);

	SetStretchBltMode(hdcWindow, HALFTONE);

	// The source DC is the entire screen, and the destination DC is the current window (HWND).
	if (!StretchBlt(hdcWindow,
		0, 0,
		rcClient.right, rcClient.bottom,
		hdcScreen,
		0, 0,
		GetSystemMetrics(SM_CXSCREEN),
		GetSystemMetrics(SM_CYSCREEN),
		SRCCOPY))
	{
		MessageBox(m_hwnd, L"StretchBlt has failed", L"Failed", MB_OK);
		goto done;
	}

	// Create a compatible bitmap from the Window DC.
	hbmScreen = CreateCompatibleBitmap(hdcWindow, rcClient.right - rcClient.left, rcClient.bottom - rcClient.top);

	if (!hbmScreen)
	{
		MessageBox(m_hwnd, L"CreateCompatibleBitmap Failed", L"Failed", MB_OK);
		goto done;
	}

	// Select the compatible bitmap into the compatible memory DC.
	SelectObject(hdcMemDC, hbmScreen);

	// Bit block transfer into our compatible memory DC.
	if (!BitBlt(hdcMemDC,
		0, 0,
		rcClient.right - rcClient.left, rcClient.bottom - rcClient.top,
		hdcWindow,
		0, 0,
		SRCCOPY))
	{
		MessageBox(m_hwnd, L"BitBlt has failed", L"Failed", MB_OK);
		goto done;
	}

	// Get the BITMAP from the HBITMAP.
	GetObject(hbmScreen, sizeof(BITMAP), &bmpScreen);

	BITMAPFILEHEADER   bmfHeader;
	BITMAPINFOHEADER   bi;

	bi.biSize = sizeof(BITMAPINFOHEADER);
	bi.biWidth = bmpScreen.bmWidth;
	bi.biHeight = bmpScreen.bmHeight;
	bi.biPlanes = 1;
	bi.biBitCount = 32;
	bi.biCompression = BI_RGB;
	bi.biSizeImage = 0;
	bi.biXPelsPerMeter = 0;
	bi.biYPelsPerMeter = 0;
	bi.biClrUsed = 0;
	bi.biClrImportant = 0;

	dwBmpSize = ((bmpScreen.bmWidth * bi.biBitCount + 31) / 32) * 4 * bmpScreen.bmHeight;

	// Starting with 32-bit Windows, GlobalAlloc and LocalAlloc are implemented as wrapper functions that 
	// call HeapAlloc using a handle to the process's default heap. Therefore, GlobalAlloc and LocalAlloc 
	// have greater overhead than HeapAlloc.
	hDIB = GlobalAlloc(GHND, dwBmpSize);
	lpbitmap = (char*)GlobalLock(hDIB);

	// Gets the "bits" from the bitmap, and copies them into a buffer 
	// that's pointed to by lpbitmap.
	GetDIBits(hdcWindow, hbmScreen, 0,
		(UINT)bmpScreen.bmHeight,
		lpbitmap,
		(BITMAPINFO*)&bi, DIB_RGB_COLORS);

	// A file is created, this is where we will save the screen capture.
	hFile = CreateFile(L"captureqwsx.bmp",
		GENERIC_WRITE,
		0,
		NULL,
		CREATE_ALWAYS,
		FILE_ATTRIBUTE_NORMAL, NULL);

	// Add the size of the headers to the size of the bitmap to get the total file size.
	dwSizeofDIB = dwBmpSize + sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER);

	// Offset to where the actual bitmap bits start.
	bmfHeader.bfOffBits = (DWORD)sizeof(BITMAPFILEHEADER) + (DWORD)sizeof(BITMAPINFOHEADER);

	// Size of the file.
	bmfHeader.bfSize = dwSizeofDIB;

	// bfType must always be BM for Bitmaps.
	bmfHeader.bfType = 0x4D42; // BM.

	WriteFile(hFile, (LPSTR)&bmfHeader, sizeof(BITMAPFILEHEADER), &dwBytesWritten, NULL);
	WriteFile(hFile, (LPSTR)&bi, sizeof(BITMAPINFOHEADER), &dwBytesWritten, NULL);
	WriteFile(hFile, (LPSTR)lpbitmap, dwBmpSize, &dwBytesWritten, NULL);

	// Unlock and Free the DIB from the heap.
	GlobalUnlock(hDIB);
	GlobalFree(hDIB);

	// Close the handle for the file that was created.
	CloseHandle(hFile);

	// Clean up.
done:
	DeleteObject(hbmScreen);
	DeleteObject(hdcMemDC);
	ReleaseDC(NULL, hdcScreen);
	ReleaseDC(m_hwnd, hdcWindow);


}

void DrawingWindow::DisplayBitmap()
{


}

// ----------------------------------------------------------------------
// Shape List
// ----------------------------------------------------------------------

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

// ----------------------------------------------------------------------
// Shapes
// ----------------------------------------------------------------------

ColorEllipse::ColorEllipse(D2D1_POINT_2F cursorPosition, D2D1_COLOR_F color)
{
	ellipse.point = cursorPosition;
	ellipse.radiusX = 1.0f;
	ellipse.radiusY = 1.0f;
	this->color = color;
}

void ColorEllipse::Draw(ID2D1RenderTarget* pRenderTarget, ID2D1SolidColorBrush* pBrush)
{
	pRenderTarget->SetTransform(D2D1::Matrix3x2F::Rotation(rotation, ellipse.point));
	pBrush->SetColor(color);
	pRenderTarget->FillEllipse(ellipse, pBrush);
	pBrush->SetColor(D2D1::ColorF(D2D1::ColorF::Black));
	pRenderTarget->DrawEllipse(ellipse, pBrush, 1.0f);
	pRenderTarget->SetTransform(D2D1::Matrix3x2F::Identity());
}

BOOL ColorEllipse::HitTest(float dipX, float dipY)
{
	const float a = ellipse.radiusX;
	const float b = ellipse.radiusY;
	const float x = dipX - ellipse.point.x;
	const float y = dipY - ellipse.point.y;
	const float xR = x * cos(-rotation * pi / 180) - y * sin(-rotation * pi / 180);
	const float yR = x * sin(-rotation * pi / 180) + y * cos(-rotation * pi / 180);

	return (xR * xR) / (a * a) + (yR * yR) / (b * b) <= 1.0f;
}

void ColorEllipse::Move(float deltaX, float deltaY)
{
	ellipse.point.x += deltaX;
	ellipse.point.y += deltaY;
}

ColorRectangle::ColorRectangle(FLOAT left, FLOAT top, FLOAT right, FLOAT bottom, FLOAT rotation, D2D1_POINT_2F relCornerOffset, D2D1_COLOR_F color)
{
	rectangle = D2D1::RectF(left, top, right, bottom);
	this->rotation = rotation;
	this->relCornerCenterOffset = relCornerOffset;
	absCornerCenterOffset = Rotate2D(this->relCornerCenterOffset, -this->rotation);
	originPosition.x = left - this->relCornerCenterOffset.x;
	originPosition.y = top - this->relCornerCenterOffset.y;
	this->color = color;
}

ColorRectangle::ColorRectangle(FLOAT width, FLOAT height, FLOAT rotation, D2D1_POINT_2F originPosition, D2D1_POINT_2F relCornerOffset, D2D1_COLOR_F color)
{
	this->rotation = rotation;
	this->originPosition = originPosition;
	this->relCornerCenterOffset = relCornerOffset;
	const float left = this->originPosition.x + this->relCornerCenterOffset.x;
	const float top = this->originPosition.y + this->relCornerCenterOffset.y;
	rectangle = D2D1::RectF(left, top, left + width, top - height);
	this->color = color;
	absCornerCenterOffset = Rotate2D(this->relCornerCenterOffset, -this->rotation);
}

ColorRectangle::ColorRectangle(D2D1_POINT_2F cursorPosition, D2D1_COLOR_F color) : color {color}
{
	rectangle.left = cursorPosition.x;
	rectangle.top = cursorPosition.y;
	rectangle.right = cursorPosition.x;
	rectangle.bottom = cursorPosition.y;
	originPosition.x = cursorPosition.x;
	originPosition.y = cursorPosition.y;
}

void ColorRectangle::Draw(ID2D1RenderTarget* pRenderTarget, ID2D1SolidColorBrush* pBrush)
{
	pRenderTarget->SetTransform(D2D1::Matrix3x2F::Rotation(rotation, originPosition));
	pBrush->SetColor(color);
	pRenderTarget->FillRectangle(rectangle, pBrush);
	pBrush->SetColor(D2D1::ColorF(D2D1::ColorF::Black));
	pRenderTarget->DrawRectangle(rectangle, pBrush, 1.0f);
	pRenderTarget->SetTransform(D2D1::Matrix3x2F::Identity());
}

BOOL ColorRectangle::HitTest(float dipX, float dipY)
{
	// Rotate
	this->Update(relCornerCenterOffset);
	D2D1_POINT_2F point{
		dipX - originPosition.x,
		dipY - originPosition.y
	};
	point = Rotate2D(point, -rotation);
	point.x = point.x + originPosition.x;
	point.y = point.y + originPosition.y;
	const float rLeft = rectangle.left;
	const float rTop = rectangle.top;
	const float rRight = rectangle.right;
	const float rBottom = rectangle.bottom;
	if (rLeft <= rRight ? (point.x >= rLeft && point.x <= rRight ) : (point.x <= rLeft && point.x >= rRight))
	{
		if (rBottom <= rTop ? (point.y >= rBottom && point.y <= rTop) : (point.y <= rBottom && point.y >= rTop))
		{
			return TRUE;
		}
	}
	return FALSE;
}

void ColorRectangle::Move(float deltaX, float deltaY, float deltaTheta)
{
	rotation += deltaTheta;
	rectangle.left += deltaX;
	rectangle.right += deltaX;
	rectangle.top += deltaY;
	rectangle.bottom += deltaY;
	absCornerCenterOffset = Rotate2D(relCornerCenterOffset, -rotation);
	absCornerOffset = Rotate2D(relCornerOffset, -rotation);
	originPosition.x += deltaX;
	originPosition.y += deltaY;
}

void ColorRectangle::UpdateRectangle(D2D1_POINT_2F absPosition, float rotation)
{
	originPosition = absPosition;
	this->rotation = rotation;
	absCornerCenterOffset = Rotate2D(relCornerCenterOffset, -rotation);
	absCornerOffset = Rotate2D(relCornerOffset, -rotation);
	rectangle.left = originPosition.x + relCornerCenterOffset.x;
	rectangle.top = originPosition.y + relCornerCenterOffset.y;
	rectangle.right = rectangle.left + relCornerOffset.x;
	rectangle.bottom = rectangle.top + relCornerOffset.y;
}

void ColorRectangle::Update(D2D1_POINT_2F relCornerCenterOffset)
{
	this->relCornerCenterOffset = relCornerCenterOffset;
	absCornerCenterOffset = Rotate2D(this->relCornerCenterOffset, -rotation);
	relCornerOffset.x = rectangle.right - rectangle.left;
	relCornerOffset.y = rectangle.bottom - rectangle.top;
	absCornerOffset = Rotate2D(relCornerOffset, -rotation);
	originPosition.x = rectangle.left - relCornerCenterOffset.x;
	originPosition.y = rectangle.top - relCornerCenterOffset.y;
}

void ColorRectangle::UpdateState(D2D1_POINT_2F topLeft, float rotation)
{
	rectangle.left = topLeft.x;
	rectangle.top = topLeft.y;
	rectangle.right = rectangle.left + relCornerOffset.x;
	rectangle.bottom = rectangle.top + relCornerOffset.y;
	originPosition.x = rectangle.left - relCornerCenterOffset.x;
	originPosition.y = rectangle.top - relCornerCenterOffset.y;
	this->rotation = rotation;
	absCornerCenterOffset = Rotate2D(relCornerCenterOffset, -rotation);
	absCornerOffset = Rotate2D(relCornerOffset, -rotation);
}

D2D1_POINT_2F Rotate2D(const D2D1_POINT_2F& vector, float rotation)
{
	D2D1_POINT_2F temp{
		cos(rotation * S_PI / 180.0) * vector.x - sin(rotation * S_PI / 180.0) * vector.y,
		sin(rotation * S_PI / 180.0) * vector.x + cos(rotation * S_PI / 180.0) * vector.y
	};
	return temp;
}

D2D1_POINT_2F Add2D(const D2D1_POINT_2F& vector1, const D2D1_POINT_2F& vector2)
{
	D2D1_POINT_2F temp{
		vector1.x + vector2.x,
		vector1.y + vector2.y
	};
	return temp;
}

// ----------------------------------------------------------------------
// GDI Bitmap
// ----------------------------------------------------------------------

BOOL GdiBitmap::CaptureScreen(HWND hwnd)
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
	BITMAP bmpScreen;

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

BOOL GdiBitmap::Display(HWND hwnd)
{
	// Prepare pixel data
	const std::string bmpPath = R"(C:\Users\Tao\LocalRepos\image-view\image-view\obstacle.bmp)";
	bitmap.Load(bmpPath);
	uint8_t* lpbitmap = bitmap.Pixels().PixelPointer();

	// Retrieve device context of windows
	HDC hdcWindow = GetDC(hwnd);

	// Drawing Bitmap
	BITMAPINFOHEADER bi;
	bi.biSize = sizeof(BITMAPINFOHEADER);
	bi.biWidth = bitmap.BmpInfo().width;
	bi.biHeight = bitmap.BmpInfo().height;
	bi.biPlanes = bitmap.BmpInfo().numColorPlanes;
	bi.biBitCount = bitmap.BmpInfo().bitsPerPixel;
	bi.biCompression = bitmap.BmpInfo().compression;
	bi.biSizeImage = bitmap.BmpInfo().imageSize;
	bi.biXPelsPerMeter = bitmap.BmpInfo().hResolution;
	bi.biYPelsPerMeter = bitmap.BmpInfo().vResolution;
	bi.biClrUsed = bitmap.BmpInfo().numColors;
	bi.biClrImportant = bitmap.BmpInfo().numImportantColors;

	SetDIBitsToDevice(hdcWindow,
		-10, 0,
		bitmap.BmpInfo().width, bitmap.BmpInfo().height,
		0, 0,
		0, bitmap.BmpInfo().height,
		lpbitmap,
		(BITMAPINFO*)&bi,
		DIB_RGB_COLORS
	);

	ReleaseDC(hwnd, hdcWindow);

	return TRUE;
}

void GdiBitmap::Draw(HWND hwnd, const POINT& centroidPos, const float& orientation)
{
	if (!hdcBmp)
	{
		return;
	}
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

	// Client device context
	HDC hdcWindow = GetDC(hwnd);

	RECT rectWindow;
	GetClientRect(hwnd, &rectWindow);

	//int y = SetDIBitsToDevice(hdcBmp,
	//	0, 0,
	//	bitmap.BmpInfo().width, bitmap.BmpInfo().height,
	//	0, 0,
	//	0, bitmap.BmpInfo().height,
	//	bitmap.Pixels().PixelPointer(),
	//	(BITMAPINFO*)&bmpInfoHeader,
	//	DIB_RGB_COLORS
	//);

	HBITMAP hBmpMask = CreateBitmap(bmpInfoHeader.biWidth, bmpInfoHeader.biHeight,
		1, 1, bitmap.RoundPixelMask(40));

	int x = PlgBlt(hdcWindow, ptParallelogram, hdcBmp,
		0, 0, bmpInfoHeader.biWidth, bmpInfoHeader.biHeight,
		hBmpMask, 0, 0);
	//int x = BitBlt(
	//	hdcWindow,
	//	0, 0,
	//	bmpInfoHeader.biWidth, bmpInfoHeader.biHeight,
	//	hdcBmp,
	//	0, 0,
	//	SRCCOPY
	//);

	//int x = StretchBlt(
	//	hdcWindow,
	//	0, 0,
	//	rectWindow.right, rectWindow.bottom,
	//	hdcBmp,
	//	0, 0, 
	//	bmpInfoHeader.biWidth, bmpInfoHeader.biHeight,
	//	SRCCOPY
	//);

	DeleteObject(hBmpMask);
	DeleteObject(hBmp);
	ReleaseDC(hwnd, hdcWindow);
}

void GdiBitmap::Release()
{
	DeleteObject(hBmp);
	DeleteObject(hdcBmp);
}

BOOL GdiBitmap::Load(std::string bmpPath)
{
	bitmap.Load(bmpPath);

	bmpInfoHeader.biSize = sizeof(BITMAPINFOHEADER);
	bmpInfoHeader.biWidth = bitmap.BmpInfo().width;
	bmpInfoHeader.biHeight = bitmap.BmpInfo().height;
	bmpInfoHeader.biPlanes = bitmap.BmpInfo().numColorPlanes;
	bmpInfoHeader.biBitCount = bitmap.BmpInfo().bitsPerPixel;
	bmpInfoHeader.biCompression = bitmap.BmpInfo().compression;
	bmpInfoHeader.biSizeImage = bitmap.BmpInfo().imageSize;
	bmpInfoHeader.biXPelsPerMeter = bitmap.BmpInfo().hResolution;
	bmpInfoHeader.biYPelsPerMeter = bitmap.BmpInfo().vResolution;
	bmpInfoHeader.biClrUsed = bitmap.BmpInfo().numColors;
	bmpInfoHeader.biClrImportant = bitmap.BmpInfo().numImportantColors;

	//hBmp = CreateBitmap(bmpInfoHeader.biWidth, bmpInfoHeader.biHeight,
	//	bmpInfoHeader.biPlanes, bmpInfoHeader.biBitCount,
	//	bitmap.Pixels().PixelPointer());

	return TRUE;
}

BOOL GdiBitmap::CreateMemDC(HWND hwnd)
{
	HDC hdcWindow = GetDC(hwnd);
	hdcBmp = CreateCompatibleDC(hdcWindow);
	//hBmp = CreateCompatibleBitmap(hdcBmp, bmpInfoHeader.biWidth, bmpInfoHeader.biHeight);
	//hBmp = CreateBitmap(bmpInfoHeader.biWidth, bmpInfoHeader.biHeight, bmpFileHeader.bmPlanes,
	//	bmpInfoHeader.biBitCount, bitmap.Pixels().PixelPointer());
	hBmp = CreateDIBitmap(hdcWindow, &bmpInfoHeader, CBM_INIT, bitmap.Pixels().PixelPointer(),
		(BITMAPINFO*)&bmpInfoHeader, DIB_RGB_COLORS);
	SelectObject(hdcBmp, hBmp);
	ReleaseDC(hwnd, hdcWindow);
	return TRUE;
}

// ------------------------------------------------- Demo ----------------------------------

int DrawingWindowDemo(HINSTANCE hInstance, HINSTANCE hPrevInstance, PWSTR pCmdLine, int nCmdShow)
{
	DrawingWindow drawingWin{};

	if (!drawingWin.Create(L"Drawing Ellipse", WS_OVERLAPPEDWINDOW, 0, CW_USEDEFAULT, CW_USEDEFAULT,
		640, 480))
	{
		return 0;
	}

	// Acceleration table
	HACCEL hAccel = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDR_ACCELERATOR1));

	ShowWindow(drawingWin.Window(), nCmdShow);

	// Message loop
	MSG msg{};
	while (GetMessage(&msg, NULL, 0, 0))
	{
		if (!TranslateAccelerator(drawingWin.Window(), hAccel, &msg))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
		
	}
	return 0;
}






