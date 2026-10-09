#include "rescue/silhouette.hpp"
#include <opencv2/imgproc.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
using namespace rescue;
namespace {
cv::Mat scene(){return cv::Mat(720,1280,CV_8UC3,cv::Scalar(150,160,170));}
const cv::Scalar kOrange(30,120,230); // BGR, hue ~10
void block(cv::Mat& img,cv::Rect r){cv::rectangle(img,r,kOrange,cv::FILLED);}
}
int main() {
    {   // lying: wide blob; upright: tall blob; box slightly looser than the blob
        auto img=scene();block(img,{400,300,160,80});block(img,{800,260,80,160});
        const float lying=orangeSilhouetteAspect(img,{390,290,180,100});
        const float upright=orangeSilhouetteAspect(img,{790,250,100,180});
        std::cerr<<"lying "<<lying<<" upright "<<upright<<"\n";
        assert(std::abs(lying-.5f)<.05f&&std::abs(upright-2.f)<.1f);
    }
    {   // a neighbouring orange block inside the box does not change the largest blob
        auto img=scene();block(img,{400,300,160,80});block(img,{570,300,20,60});
        assert(std::abs(orangeSilhouetteAspect(img,{390,290,220,100})-.5f)<.05f);
    }
    {   // unusable inputs keep the box-only verdict
        auto img=scene();
        assert(orangeSilhouetteAspect(img,{400,300,160,80})<0);              // no orange
        block(img,{400,300,4,4});assert(orangeSilhouetteAspect(img,{390,290,180,100})<0); // speck
        assert(orangeSilhouetteAspect(cv::Mat(),{0,0,50,50})<0);
        assert(orangeSilhouetteAspect(img,{2000,2000,50,50})<0);
        assert(orangeSilhouetteAspect(img,{0,0,5,5})<0);
        cv::Mat gray(720,1280,CV_8UC1,cv::Scalar(0));assert(orangeSilhouetteAspect(gray,{0,0,100,100})<0);
    }
    {   // green and blue blocks are not orange
        auto img=scene();cv::rectangle(img,{400,300,160,80},cv::Scalar(60,200,60),cv::FILLED);
        cv::rectangle(img,{700,300,80,160},cv::Scalar(230,160,40),cv::FILLED);
        assert(orangeSilhouetteAspect(img,{390,290,180,100})<0&&orangeSilhouetteAspect(img,{690,290,100,180})<0);
    }
    {   // decision rule
        assert(silhouetteAdjustedDiff(-.02f,.7f)==-.11f);       // wide: pushed to clearly lying
        assert(silhouetteAdjustedDiff(-.30f,.7f)==-.30f);       // already clearly lying: untouched
        assert(silhouetteAdjustedDiff(-.02f,1.4f)==.10f);       // tall: clearly upright
        assert(silhouetteAdjustedDiff(.2f,1.4f)==.2f);
        for(float a:{.9f,1.0f,1.15f})assert(silhouetteAdjustedDiff(-.03f,a)==-.03f); // grey zone defers to the box fit
        for(float a:{-1.f,0.f,NAN,INFINITY})assert(silhouetteAdjustedDiff(-.03f,a)==-.03f); // unavailable or non-finite: ignored
        const float adjusted=silhouetteAdjustedDiff(-.02f,.7f);
        assert(adjusted<-.06f);                                  // below the confirm line: candidate
    }
    std::cout<<"silhouette tests passed\n";
}
