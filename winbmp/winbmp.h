#ifndef WINBMP_H
#define WINBMP_H

#ifdef WINBMP_EXPORTS
#define WINBMP_API __declspec(dllexport)
#else
#define WINBMP_API __declspec(dllimport)
#endif

#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <cmath>

/*
The BMP file format is described in :
https://en.wikipedia.org/wiki/BMP_file_format#File_structure
*/

typedef unsigned char uint8_t;
typedef unsigned int uint32_t;
typedef signed int int32_t;
typedef unsigned short int uint16_t;

struct WINBMP_API  Pixel
{
	static const int MIN_RGB = 0;
	static const int MAX_RGB = 255;
	uint8_t red, green, blue;
	Pixel() : red{ 0 }, green{ 0 }, blue{ 0 } {}
	Pixel(uint8_t r, uint8_t g, uint8_t b) : red{ r }, green{ g }, blue{ b } {}
};

struct WINBMP_API PixelMatrix
{
	std::vector<std::vector<Pixel>> pixelMatrix;
	std::vector<Pixel> pixelArray;
	uint8_t* pixelPointer{nullptr};

	PixelMatrix() = default;
	~PixelMatrix() { delete[] pixelPointer; }
	std::vector<std::vector<Pixel>>& operator()() { return pixelMatrix; }
	std::vector<Pixel>& PixelArray();
	uint8_t* PixelPointer();
	auto PixelNum() { if (!pixelArray.empty()) return pixelArray.size(); }
	void Clear() { pixelMatrix.clear(); pixelArray.clear(); }
	bool IsImage();
	void FromPointer(uint8_t* pixelPointer, int height, int width);
};

struct WINBMP_API BmpHeader
{
	// Header (14 bytes)
	uint32_t fileSize{ 0 };
	uint32_t reserved{ 0 };
	uint32_t offset{ 54 };
};

struct WINBMP_API BitmapInfoHeader
{
	// DIB Header: BITMAPINFOHEADER (40 bytes)
	uint32_t headerSize{ 40 };
	int32_t width{ 0 };
	int32_t height{ 0 };
	uint16_t numColorPlanes{ 1 };
	uint16_t bitsPerPixel{ 24 };
	uint32_t compression{ 0 };
	uint32_t imageSize{ 0 };
	uint32_t hResolution{ 2835 };
	uint32_t vResolution{ 2835 };
	uint32_t numColors{ 0 };
	uint32_t numImportantColors{ 0 };
	// Color Table (only present if bitsPerPixel < 8)
};

class WINBMP_API Bmp // Deprecated
{
private:
	unsigned char m_signature[2]{ 'B', 'M' };
	BmpHeader m_bmpHeader;
	BitmapInfoHeader m_bmpInfoHeader;
	PixelMatrix m_pixelMatrix;
	uint8_t* m_bmpBlackMask{ nullptr };
	uint8_t* m_bmpRoundMask{ nullptr };
	bool UpdateBmpInfo();				// Update signature, header and BITMAPINFOHEADER

public:
	Bmp() = default;
	Bmp(std::string bmpPath);
	~Bmp() { delete[] m_bmpBlackMask; }
	void Load(std::string bmpPath);
	void Save(std::string savePath);
	void PrintHeader();
	PixelMatrix& Pixels() { return m_pixelMatrix; }
	//uint8_t*& BlackPixelMask();
	uint8_t*& RoundPixelMask(float radius);
	BitmapInfoHeader& BmpInfo() { return m_bmpInfoHeader; }
	
	const static int H_RESOLUTION = 2835;	// Pixel per Inch
	const static int V_RESOLUTION = 2835;	// Pixel per inch
};


#endif // !WINBMP_H