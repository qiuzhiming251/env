/**

* @file draw_point.h
* @author xu.xuefeng1
* @brief  纯头 画散点图实现 保存Pnm格式 用于单元测试小工程
* 运行环境中（docker）确保执行过   sudo apt install pnmtopng
* @note 使用方法 ：
* 1. 初始化一个ppm对象，初始化参数还包括画布的宽度 高度 栅格精度 图片的filepath。
* 2.  ppm对象调用DrawPoint函数，输入一个点，画布上画一个点,输入参数点坐标和点对应的颜色。
*
    // 创建一个 200x200 的 PPM 图像，精度 0.1m，文件路径 "output.ppm"
    PPMImage ppm(200, 200, 0.1, "output.ppm");

    // 在 (50, 50) 位置绘制一个红色点
    ppm.DrawPoint(50, 50, {255, 0, 0});

    // 在 (100, 100) 位置绘制一个绿色点
    ppm.DrawPoint(100, 100, {0, 255, 0});

    // 在 (150, 150) 位置绘制一个蓝色点
    ppm.DrawPoint(150, 150, {0, 0, 255});

    // 手动调用 save 方法保存图像
    ppm.save();
*
* @version 0.1
* @date 21/11/29
  
*/
#ifndef DRAW_POINT_H
#define DRAW_POINT_H
#include <fstream>
#include <cmath>
#include <filesystem>
#include <vector>
#include <map>
#include <string>
#include <future>
#include <stdio.h>
#include <stdlib.h>
#include <random>
#include <chrono>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <iostream>
#define WIDTH  1000
#define HEIGHT  3000
#define Y_OFFSET  2000 //栅格（图像坐标系下 车体坐标系原点的位置）  
#define X_OFFSET  500
#define DRAW_LINE_WIDTH  3
#define DRAW_PRECISION  0.1f
namespace ppm {

class PPMImage {
public:
    // 构造函数，初始化图像的宽度、高度、精度和文件路径
    static int ppmCount;
    PPMImage() {
        std::ostringstream oss;
        oss << std::setw(4) << std::setfill('0') << ppmCount;
        // filePath_ = "modules/mpc/mpc_drv_ctap_fusion_lane/src/test/data/datadump/" + oss.str() +".ppm";
        filePath_ = "/apollo/modules/perception/env/src/test/data/datadump/" + oss.str() +".ppm";
        precision_ = DRAW_PRECISION;
        lineWidth_ = DRAW_LINE_WIDTH;
        offsetX_ = X_OFFSET;
        offsetY_ = Y_OFFSET;
        width_ = WIDTH;
        height_ = HEIGHT;
        // memset(&image_[0][0][0], 0, width_ * height_ * 3);
        // image_.resize(HEIGHT, std::vector<std::vector<u_char>>(width_, std::vector<u_char>(3, 0)));
        image_= new unsigned char[height_ * width_ * 3]{0};

    };
    // 析构函数
    ~PPMImage() {
        delete[] image_;
    }

    // 在图像上绘制一个点
    inline void DrawPoint(const float &vcsX, const float &vcsY, const int &colorIdx) {
        int y = static_cast<int>(offsetY_ - vcsX / precision_);
        int x = static_cast<int>(offsetX_ - vcsY / precision_);

        for (int i = -lineWidth_; i <= lineWidth_; ++i) {
            for (int j = -lineWidth_; j <= lineWidth_; ++j) {
                int dx = i * i;
                int dy = j * j;
                if (dx + dy <= lineWidth_ * lineWidth_) {
                    int newX = x + i;
                    int newY = y + j;
                    if (newX >= 0 && newX < width_  && newY >= 0 && newY < height_  && colorIdx <15) {
                        image_[(newY*width_ + newX)*3]     =    colors_[colorIdx][0];
                        image_[(newY*width_ + newX)*3 +1]  =    colors_[colorIdx][1];
                        image_[(newY*width_ + newX)*3 +2 ] =    colors_[colorIdx][2];
                    } else {
                         newX = debugHelper;
                        std::cerr << "vcsX ===" << vcsX << " , " << vcsY<<  std::endl;
                    }
                }
            }
        }


    }
    inline void DrawPointOther(const float &vcsX, const float &vcsY, const int &colorIdx) {
        int y = static_cast<int>(offsetY_ - vcsX / precision_);
        int x = static_cast<int>(offsetX_ - vcsY / precision_);



                    if (x >= 0 && x < width_  && y >= 0 && y < height_  && colorIdx <15) {
                        image_[(y*width_ + x)*3]     =    colors_[colorIdx][0];
                        image_[(y*width_ + x)*3 +1]  =    colors_[colorIdx][1];
                        image_[(y*width_ + x)*3 +2 ] =    colors_[colorIdx][2];
                    } else {
                        std::cerr << "DrawPoint ===" << x <<  ">:  " << width_ << y << ">  " << height_ << "coloridx: " << colorIdx <<  std::endl;
                    }



    }



inline void DrawSig(int sigIndex, float startX, float startY, int colorIdx) 
{
    int charWidth = 5; // 每个字符的宽度
    int charHeight = 8; // 每个字符的高度
    for (int h = 0; h < charHeight; ++h) {
    for (int w = 0; w < charWidth; ++w) {
        if (sigChars[sigIndex][h * charWidth + w] == 1) {
            // DrawPoint(startX + i * (charWidth + spaceBetweenChars) + x, startY + y, colorIdx);
            // h w 转化为图像上点的坐标

            int x = static_cast<int>(offsetX_ - (startY/precision_) + (charWidth) + w);
            int y = static_cast<int>(offsetY_ - (startX/precision_) + h);
            int newX = x;
            int newY = y;
            if (newX >= 0 && newX < width_  && newY >= 0 && newY < height_  && colorIdx <15) {
                image_[(newY*width_ + newX)*3]     =    colors_[colorIdx][0];
                image_[(newY*width_ + newX)*3 +1]  =    colors_[colorIdx][1];
                image_[(newY*width_ + newX)*3 +2 ] =    colors_[colorIdx][2];
            } else {
                std::cerr << " DrawSig Point (" << newX << ", " << newY << ") is out of bounds." << std::endl;
            }
            }

            
        }
    }
}
inline void DrawNumber(int number, float startX, float startY, int colorIdx) {
    std::string numberStr = std::to_string(number);
    int charWidth = 10; // 每个字符的宽度
    int charHeight = 12; // 每个字符的高度
    int spaceBetweenChars = 4; // 字符之间的间距

    for (size_t i = 0; i < numberStr.size(); ++i) {
        int digit = numberStr[i] - '0'; // 将字符转换为数字
        for (int h = 0; h < charHeight; ++h) {
            for (int w = 0; w < charWidth; ++w) {
                if (digitChars[digit][h * charWidth + w] == 1) {
                    // DrawPoint(startX + i * (charWidth + spaceBetweenChars) + x, startY + y, colorIdx);
                    // h w 转化为图像上点的坐标

                    int x = static_cast<int>(offsetX_ - (startY/precision_) + i * (charWidth + spaceBetweenChars) + w);
                    int y = static_cast<int>(offsetY_ - (startX/precision_) + h);
                    int newX = x;
                    int newY = y;
                    if (newX >= 0 && newX < width_  && newY >= 0 && newY < height_  && colorIdx <15) {
                        image_[(newY*width_ + newX)*3]     =    colors_[colorIdx][0];
                        image_[(newY*width_ + newX)*3 +1]  =    colors_[colorIdx][1];
                        image_[(newY*width_ + newX)*3 +2 ] =    colors_[colorIdx][2];

                        
                    } else {
                        std::cerr << " DrawNumber Point==== (" << newX << ", " << newY << ") is out of bounds." << std::endl;
                        //禁止优化
                        newX = debugHelper;
                    }
                 }

                    
                }
            }
        }
}
    // 保存图像到文件
inline void Save() {
    FILE *f6 = fopen(filePath_.c_str(), "w");
    if (f6==NULL) {
        printf("FILE error\n");
        return;
    }

    // 写入 PPM 文件头
    fprintf(f6, "P6\n");
    fprintf(f6, "%d %d\n", width_, height_);
    fprintf(f6, "255\n");
    fwrite(image_, sizeof(u_char), width_ * height_ * 3, f6);
    fclose(f6);
    ppmCount++;

}
inline void ShuffleColorMap() {
    auto seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::shuffle(colors_.begin(), colors_.end(), std::default_random_engine(seed));
}
private:
    int width_;
    int height_;
    int offsetX_;
    int offsetY_;
    int lineWidth_;
    double precision_;
    std::string filePath_;
    u_char *image_= nullptr;
    volatile int debugHelper;
    // std::vector<std::vector<std::vector<u_char>>>image_(10, std::vector<std::vector<u_char>> (20, std::vector<u_char>(3, 0)));


    std::vector<std::vector<u_char>> colors_ = {
        {255, 255, 0}, // 鲜黄色
        {0, 255, 0},   // 鲜绿色
        {255, 165, 0}, // 鲜橙色
        {0, 0, 255},   // 鲜蓝色
        {255, 0, 0},   // 鲜红色
        {255, 0, 255}, // 鲜品红
        {0, 255, 255}, // 鲜青色
        {255, 192, 203}, // 鲜粉红色
        {75, 0, 130}, // 鲜靛蓝色
        {255, 105, 180}, // 鲜热粉色
        {255, 140, 50}, // 鲜珊瑚色
        {0, 255, 127}, // 鲜酸橙绿
        {173, 255, 47}, // 鲜黄绿色
        {30, 144, 255}, // 鲜深天蓝
        {135, 206, 235}, // 鲜天蓝色
        {255, 218, 185}, // 鲜贝壳色
        {255, 182, 193}, // 鲜浅紫色
        {255, 160, 122}, // 鲜浅珊瑚色
        {255, 69, 0}, // 鲜番茄色
        {255, 127, 80}, // 鲜橙色
        {255, 215, 0}, // 鲜金色
        {0, 255, 255}, // 鲜青色（亮）
        {255, 255, 255}, // 白色（最亮）
    };
    const std::vector<std::vector<int>> sigChars = {
        // 0
       // 0
    {0, 1, 0, 1, 0,
     0, 1, 0, 1, 0,
     0, 1, 0, 1, 0,
     1, 1, 1, 1, 1,
     1, 1, 1, 1, 1,
     0, 1, 0, 1, 0,
     0, 1, 0, 1, 0,
     0, 1, 0, 1, 0},
    {1, 0, 0, 0, 1,
     0, 1, 0, 1, 0,
     0, 0, 1, 0, 0,
     0, 0, 1, 0, 0,
     0, 0, 1, 0, 0,
     0, 0, 1, 0, 0,
     0, 1, 0, 1, 0,
     1, 0, 0, 0, 1}};
    const std::vector<std::vector<int>> digitChars = {
    {
     1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    },
    {
     0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    },
    {
     1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    },
    {
     1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    },
    {
     1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    },
    // 5
    {
     1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    },
    {
     1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    },
    {
     0, 0, 0, 0, 1, 1, 1, 1, 1, 1,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    },

    // 8
    {
     1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    },
    {
     1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
     1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    }
    };

};
} // namespace draw_point
#endif // DRAW_POINT_H