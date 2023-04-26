#include "pch.h"
#include "winbmp.h"

bool Bmp::UpdateBmpInfo()
{
    if (m_pixelMatrix.IsImage())
    {
        m_signature[0] = 'B';
        m_signature[1] = 'M';

        m_bmpHeader.offset = sizeof(m_signature) + sizeof(BmpHeader) + sizeof(BitmapInfoHeader);
        m_bmpHeader.fileSize = m_bmpHeader.offset + m_pixelMatrix().size()
            * (m_pixelMatrix()[0].size() * 3 + m_pixelMatrix()[0].size() % 4);

        m_bmpInfoHeader.headerSize = sizeof(BitmapInfoHeader);
        m_bmpInfoHeader.width = m_pixelMatrix()[0].size();
        m_bmpInfoHeader.height = m_pixelMatrix().size();
        m_bmpInfoHeader.numColorPlanes = 1;
        m_bmpInfoHeader.bitsPerPixel = 24;
        m_bmpInfoHeader.compression = 0;
        m_bmpInfoHeader.imageSize = 0;
        m_bmpInfoHeader.hResolution = 2835;
        m_bmpInfoHeader.vResolution = 2835;
        m_bmpInfoHeader.numColors = 0;
        m_bmpInfoHeader.numImportantColors = 0;
        
        return true;
    }
    
    return false;
}

Bmp::Bmp(std::string bmpPath)
{
    Load(bmpPath);
}

void Bmp::Load(std::string bmpPath)
{
    // Open BMP file
    std::fstream bmpFile(bmpPath.c_str(), std::ios::in | std::ios::binary | std::ios::ate);
    if (!bmpFile.is_open())
    {
        std::cerr << "Failed to open file at " << bmpPath << "\n";
    }
    else
    {
        //int fileSize = bmpFile.tellg();

        bmpFile.seekg(0, std::ios::beg);

        // Read signature
        bmpFile.read(reinterpret_cast<char*>(m_signature), sizeof(m_signature));
        if (m_signature[0] != 'B' || m_signature[1] != 'M')
        {
            std::cout << "Signature is not BM " << "\n";
            return;
        }

        // Read Header
        bmpFile.read(reinterpret_cast<char*>(&m_bmpHeader), sizeof(m_bmpHeader));

        // Read BITMAPINFOHEADER
        bmpFile.read(reinterpret_cast<char*>(&m_bmpInfoHeader), sizeof(m_bmpInfoHeader));

        bool flip = true;
        if (m_bmpInfoHeader.height < 0)
        {
            flip = false;
            m_bmpInfoHeader.height = -m_bmpInfoHeader.height;
        }
        if (m_bmpInfoHeader.bitsPerPixel != 24)
        {
            std::cout << "File uses" << m_bmpInfoHeader.bitsPerPixel <<
                "bits per pixel. Bitmap only supports 25 bit.\n";
            return;
        }
        if (m_bmpInfoHeader.compression != 0)
        {
            std::cout << "Compression not supported.";
        }

        // Read pixel array
        m_pixelMatrix.Clear();
        bmpFile.seekg(m_bmpHeader.offset);
        for (int row{ 0 }; row < m_bmpInfoHeader.height; ++row)
        {
            std::vector<Pixel> rowData;

            for (int col{ 0 }; col < m_bmpInfoHeader.width; ++col)
            {
                int blue = bmpFile.get();
                int green = bmpFile.get();
                int red = bmpFile.get();
                rowData.push_back(Pixel(red, green, blue));
            }
            bmpFile.seekg(m_bmpInfoHeader.width % 4, std::ios::cur);

            if (flip)
            {
                m_pixelMatrix().insert(m_pixelMatrix().begin(), rowData);
            }
            else
            {
                m_pixelMatrix().push_back(rowData);
            }
        }

        //PrintHeader();

        //std::cout << "Pixel array size: " << m_pixelMatrix.PixelArray().size() << '\n';

        //std::cout << "Is image ? " << std::boolalpha << m_pixelMatrix.IsImage() << '\n';

        bmpFile.close();
    }
}

void Bmp::Save(std::string savePath)
{
    std::fstream bmpFile(savePath.c_str(), std::ios::out | std::ios::binary);

    if (bmpFile.fail())
    {
        std::cout << "File could not be opened.\n";
        return;
    }
    else if (!m_pixelMatrix.IsImage())
    {
        std::cout << "Invalid image.\n";
        return;
    }
    else if (!UpdateBmpInfo())
    {
        std::cout << "Bitmap info update error.\n";
        return;
    }
    else
    {
        // Signature
        unsigned char signature[2]{ 'B', 'M' };
        bmpFile.write(reinterpret_cast<char*>(&signature), sizeof(signature));
        
        // Header
        bmpFile.write(reinterpret_cast<char*>(&m_bmpHeader), sizeof(m_bmpHeader));

        // Info Header
        bmpFile.write(reinterpret_cast<char*>(&m_bmpInfoHeader), sizeof(m_bmpInfoHeader));

        // Pixel Matrix
        for (int row = m_pixelMatrix().size() - 1 ; row >= 0; --row)
        {
            for (int col{ 0 }; col < m_pixelMatrix()[row].size(); ++col)
            {
                Pixel& pixel{ m_pixelMatrix()[row][col] };

                bmpFile.put(static_cast<uint8_t>(pixel.blue));
                bmpFile.put(static_cast<uint8_t>(pixel.green));
                bmpFile.put(static_cast<uint8_t>(pixel.red));
            }
            for (int i{ 0 }; i < m_pixelMatrix()[row].size() % 4; ++i)
            {
                bmpFile.put(0);
            }
        }
        bmpFile.close();
    }
}

void Bmp::PrintHeader()
{
    std::cout << "Signature: " << m_signature[0] << m_signature[1] << '\n';
    std::cout << "File Size: " << m_bmpHeader.fileSize << '\n';
    std::cout << "Reserved: " << m_bmpHeader.reserved << '\n';
    std::cout << "Offset: " << m_bmpHeader.offset << '\n';

    std::cout << "Header Size: " << m_bmpInfoHeader.headerSize << '\n';
    std::cout << "Width: " << m_bmpInfoHeader.width << '\n';
    std::cout << "Height: " << m_bmpInfoHeader.height << '\n';
    std::cout << "Number of Color Planes: " << m_bmpInfoHeader.numColorPlanes << '\n';
    std::cout << "Bits per Pixel: " << m_bmpInfoHeader.bitsPerPixel << '\n';
    std::cout << "Compression: " << m_bmpInfoHeader.compression << '\n';
    std::cout << "Image Size: " << m_bmpInfoHeader.imageSize << '\n';
    std::cout << "Horizontal Resolution: " << m_bmpInfoHeader.hResolution << '\n';
    std::cout << "Vertical Resolution: " << m_bmpInfoHeader.vResolution << '\n';
    std::cout << "Number of Color: " << m_bmpInfoHeader.numColors << '\n';
    std::cout << "Number of Important Color: " << m_bmpInfoHeader.numImportantColors << '\n';
}

//uint8_t*& Bmp::BlackPixelMask()
//{
//    if (!m_bmpBlackMask && Pixels().IsImage())
//    {
//        m_bmpBlackMask = new uint8_t[Pixels().PixelNum() / 8 + 1];
//        int idx = 0;
//        uint8_t bitMask = 0x01;
//        int bitCount = 0;
//
//        for (int row = height - 1; row >= 0; --row)
//        {
//            if (bitMask != 0x80)
//            {
//                bitMask = 0x80;
//                ++idx;
//            }
//            *(m_bmpRoundMask + idx) = 0x00;
//
//            for (int col = 0; col < width; ++col)
//            {
//                float r = pow((float)col - centerX, 2) + pow((float)row - centerY, 2);
//                if (r < radiusSquared)
//                {
//                    *(m_bmpRoundMask + idx) |= bitMask;
//                }
//
//                if (bitMask == 0x01)
//                {
//                    bitMask = 0x80;
//                    ++idx;
//                    *(m_bmpRoundMask + idx) = 0x00;
//                }
//                else
//                {
//                    bitMask >>= 1;
//                }
//            }
//        }
//
//        for (Pixel pixel : Pixels().PixelArray())
//        {
//            if (bitMask == 0x01)
//            {
//                *(m_bmpBlackMask + idx) = 0;
//            }
//
//            if (pixel.blue < 2 && pixel.green < 2 && pixel.red < 2)
//            {
//                *(m_bmpBlackMask + idx) |= 0x00;
//            }
//            else
//            {
//                *(m_bmpBlackMask + idx) |= bitMask;
//            }
//
//            if (bitMask == 0x80)
//            {
//                bitMask = 0x01;
//                ++idx;
//            }
//            else
//            {
//                bitMask <<= 1;
//            }
//        }
//    }
//    return m_bmpBlackMask;
//}

uint8_t*& Bmp::RoundPixelMask(float radius)
{
    if (!m_bmpRoundMask && Pixels().IsImage())
    {
        const auto height = Pixels().pixelMatrix.size();
        const auto width = Pixels().pixelMatrix.front().size();
        const auto nPlanes = 1;
        const auto bitPerPixel = 1;
        const auto bufferSize = (((width * nPlanes * bitPerPixel + 15) >> 4) << 1) * height;
        
        const float centerX = width / 2.0f;
        const float centerY = height / 2.0f;
        const float radiusSquared = radius * radius;

        m_bmpRoundMask = new uint8_t[bufferSize];
        int idx = 0;
        uint8_t bitMask = 0x80;
        for (int row = height - 1; row >= 0; --row)
        {
            if (bitMask != 0x80)
            {
                bitMask = 0x80;
                ++idx;
            }
            *(m_bmpRoundMask + idx) = 0x00;

            for (int col= 0 ; col < width; ++col)
            {
                float r = pow((float)col - centerX, 2) + pow((float)row - centerY, 2);
                if (r < radiusSquared)
                {
                    *(m_bmpRoundMask + idx) |= bitMask;
                }

                if (bitMask == 0x01)
                {
                    bitMask = 0x80;
                    ++idx;
                    *(m_bmpRoundMask + idx) = 0x00;
                }
                else
                {
                    bitMask >>= 1;
                }
            }
        }
    }
    return m_bmpRoundMask;
}

std::vector<Pixel>& PixelMatrix::PixelArray()
{

    if (!pixelMatrix.empty() && pixelArray.empty())
    {
        const int height = pixelMatrix.size();
        const int width = pixelMatrix.front().size();

        for (int row{ 0 }; row < height; ++row)
        {
            for (int col{ 0 }; col < width; ++col)
            {
                pixelArray.push_back(pixelMatrix[row][col]);
            }
        }
    }
    return pixelArray;
}

uint8_t* PixelMatrix::PixelPointer()
{
    if (!pixelPointer && !pixelMatrix.empty())
    {
        const int height = pixelMatrix.size();
        const int width = pixelMatrix.front().size();
        const int size = height * width * 3;
        pixelPointer = new uint8_t[size];
        
        int idx = 0;
        for (int row{ height - 1 }; row >= 0; --row)
        {
            for (int col{ 0 }; col < width; ++col)
            {
                *(pixelPointer + idx) = pixelMatrix[row][col].blue;
                ++idx;
                *(pixelPointer + idx) = pixelMatrix[row][col].green;
                ++idx;
                *(pixelPointer + idx) = pixelMatrix[row][col].red;
                ++idx;
            }
        }
    }
    return pixelPointer;
}

bool PixelMatrix::IsImage()
{
    if (pixelMatrix.empty()) 
    {
        return false;
    }
    int height = pixelMatrix.size();

    if (pixelMatrix.front().empty())
    {
        return false;
    }
    int width = pixelMatrix.front().size();

    for (int row{ 0 }; row < height; ++row)
    {
        if (pixelMatrix[row].size() != width)
        {
            return false;
        }
        for (int col{ 0 }; col < width; ++col)
        {
            Pixel pixel = pixelMatrix[row][col];
            if (pixel.red > Pixel::MAX_RGB || pixel.red < Pixel::MIN_RGB ||
                pixel.green > Pixel::MAX_RGB || pixel.green < Pixel::MIN_RGB ||
                pixel.blue > Pixel::MAX_RGB || pixel.blue < Pixel::MIN_RGB)
            {
                return false;
            }
        }
        return true;
    }

  

    return false;
}

void PixelMatrix::FromPointer(uint8_t* pixelPointer, int height, int width)
{
    if (pixelPointer)
    {
        Clear();
        const int size = height * width * 3;
        int idx = 0;
        for (int row{ 0 }; row < height; ++row)
        {
            std::vector<Pixel> rowData;
            for (int col{ 0 }; col < width; ++col)
            {
                int blue = *(pixelPointer + idx);
                ++idx;
                int green = *(pixelPointer + idx);
                ++idx;
                int red = *(pixelPointer + idx);
                ++idx;
                rowData.push_back(Pixel(red, green, blue));
            }
            pixelMatrix.insert(pixelMatrix.begin(), rowData);
        }
    }
}
