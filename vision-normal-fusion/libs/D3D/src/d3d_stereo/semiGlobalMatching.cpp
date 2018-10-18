/*///////////////////////////////////////////////////////////////////////////////////////
//
// THIS IS A MODIFICATION OF OPENCV SOURCE CODE
//
//*/

/*///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//*/

/*
 This is a variation of
 "Stereo Processing by Semiglobal Matching and Mutual Information"
 by Heiko Hirschmuller.

 This version was adapted from the OpenCV implementation to use
 a matching cost volume instead of two stereo rectified images.

 */

#include <limits.h>
#include "semiGlobalMatching.h"
#include <iostream>
#include <d3d_base/exception.h>
#include <opencv2/core/internal.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Dense>

using std::cout;
using std::endl;
using Eigen::Matrix;

using namespace D3D;

SemiGlobalMatching::SemiGlobalMatching()
{
    outputDepthMapEnabled = true;
    outputLabelsEnabled =true;
    outputUniquenessRatiosEnabled = false;
    outputCostsEnabled = false;
    subPixelEnabled = true;
//    minDisparity = numberOfDisparities = 0;
//    SADWindowSize = 0;
    P1 = 5; P2 = 8;
//    disp12MaxDiff = 0;
//    preFilterCap = 0;
//    uniquenessRatio = 0;
//    speckleWindowSize = 0;
//    speckleRange = 0;
    fullDP = true;
}


SemiGlobalMatching::SemiGlobalMatching(int _P1, int _P2, bool _fullDP )
{
    outputDepthMapEnabled = true;
    outputLabelsEnabled =true;
    outputUniquenessRatiosEnabled = false;
    outputCostsEnabled = false;
    subPixelEnabled = true;
    P1 = _P1;
    P2 = _P2;
    fullDP = _fullDP;
}

/*
  This is an adaptation to the original open cv code to enable the semi global matching to mach on
  a cost volume acquired by plane sweep. Thus the comments might make no sense sometimes and also too much
  memory is allocated. But the implementation seems to be still fast.

 computes disparity for "roi" in img1 w.r.t. img2 and write it to disp1buf.
 that is, disp1buf(x, y)=d means that img1(x+roi.x, y+roi.y) ~ img2(x+roi.x-d, y+roi.y).
 minD <= d < maxD.
 disp2full is the reverse disparity map, that is:
 disp2full(x+roi.x,y+roi.y)=d means that img2(x+roi.x, y+roi.y) ~ img1(x+roi.x+d, y+roi.y)

 note that disp1buf will have the same size as the roi and
 disp2full will have the same size as img1 (or img2).
 On exit disp2buf is not the final disparity, it is an intermediate result that becomes
 final after all the tiles are processed.

 the disparity in disp1buf is written with sub-pixel accuracy
 (4 fractional bits, see CvStereoSGBM::DISP_SCALE),
 using quadratic interpolation, while the disparity in disp2buf
 is written as is, without interpolation.

 disp2cost also has the same size as img1 (or img2).
 It contains the minimum current cost, used to find the best disparity, corresponding to the minimal cost.
 */
void SemiGlobalMatching::matchFromPlaneSweep(const Grid<float>& costVolume, const Grid<Eigen::Vector4d>& planes, const CameraMatrix<double>& cam, float costThresh)
{
    Matrix<double, 3, 3> refKinvT = Matrix<double, 3, 3>::Zero();
    if (outputDepthMapEnabled)
    {
        result = DepthMap<float, double>(costVolume.getWidth(), costVolume.getHeight(), cam);
        refKinvT  = cam.getK().inverse().transpose();
    }
//    for(int z=0; z<costVolume.getDepth(); ++z)
//    {
//        cout<<costVolume(847,247,z)<<" - ";
//    }
//    cout<<endl;
    if (outputUniquenessRatiosEnabled)
    {
        uniquenessRatios = Grid<float>(costVolume.getWidth(), costVolume.getHeight(), 1 ,1);
    }

    if (outputCostsEnabled)
    {
        costs = Grid<float>(costVolume.getWidth(), costVolume.getHeight(), 1, 0);
    }

    cv::Mat disp1(costVolume.getHeight(), costVolume.getWidth(), CV_16S );

#if CV_SSE2
    static const uchar LSBTab[] =
    {
        0, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0, 4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
        5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0, 4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
        6, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0, 4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
        5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0, 4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
        7, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0, 4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
        5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0, 4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
        6, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0, 4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
        5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0, 4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0
    };

    volatile bool useSIMD = cv::checkHardwareSupport(CV_CPU_SSE2);
#endif

    const int ALIGN = 16;
    const int DISP_SCALE = SemiGlobalMatching::DISP_SCALE;
    const CostType MAX_COST = SHRT_MAX;

    int minD = 0, maxD = costVolume.getDepth();
    int k, width = costVolume.getWidth(), height = costVolume.getHeight();
    int minX1 = 0, maxX1 = width;
    int D = maxD - minD, width1 = costVolume.getWidth();
    int INVALID_DISP = minD - 1, INVALID_DISP_SCALED = INVALID_DISP*DISP_SCALE;
    int npasses = fullDP ? 2 : 1;

    if( minX1 >= maxX1 )
    {
        disp1 = cv::Scalar::all(INVALID_DISP_SCALED);
        return;
    }

    if (D % 16 != 0)
    {
        D3D_THROW_EXCEPTION("Number of labels must be a multiple of 16 because of the alignment.")
    }

    // NR - the number of directions. the loop on x below that computes Lr assumes that NR == 8.
    // if you change NR, please, modify the loop as well.
    int D2 = D+16, NRD2 = NR2*D2;

    // the number of L_r(.,.) and min_k L_r(.,.) lines in the buffer:
    // for 8-way dynamic programming we need the current row and
    // the previous row, i.e. 2 rows in total
    const int NLR = 2;
    const int LrBorder = NLR - 1;

    // for each possible stereo match (img1(x,y) <=> img2(x-d,y))
    // we keep pixel difference cost (C) and the summary cost over NR directions (S).
    // we also keep all the partial costs for the previous line L_r(x,d) and also min_k L_r(x, k)
    size_t costBufSize = width1*D;
    size_t CSBufSize = costBufSize*(fullDP ? height : 1);
    size_t minLrSize = (width1 + LrBorder*2)*NR2, LrSize = minLrSize*D2;
    int SH2 = 0;
    int hsumBufNRows = SH2*2 + 2;
    size_t totalBufSize = (LrSize + minLrSize)*NLR*sizeof(CostType) + // minLr[] and Lr[]
            costBufSize*(hsumBufNRows + 1)*sizeof(CostType) +  // hsumBuf, pixdiff
            CSBufSize*2*sizeof(CostType) + 1024;// C, S
    if (totalBufSize > (unsigned int)std::numeric_limits<int>::max())
    {
        //std::cout << "totalBufSize = " << totalBufSize << std::endl;
        D3D_THROW_EXCEPTION("Image too big for SGM");
    }
    if( !buffer.data || !buffer.isContinuous() ||
            buffer.cols*buffer.rows*buffer.elemSize() < totalBufSize )
        buffer.create(1, (int)totalBufSize, CV_8U);

    // summary cost over different (nDirs) directions
    CostType* Cbuf = (CostType*)cv::alignPtr(buffer.data, ALIGN);
    CostType* Sbuf = Cbuf + CSBufSize;
    CostType* hsumBuf = Sbuf + CSBufSize;
    CostType* pixDiff = hsumBuf + costBufSize*hsumBufNRows;

    // add P2 to every C(x,y). it saves a few operations in the inner loops
    for( k = 0; k < width1*D; k++ )
        Cbuf[k] = (CostType)P2;

    for( int pass = 1; pass <= npasses; pass++ )
    {
        int x1, y1, x2, y2, dx, dy;

        if( pass == 1 )
        {
            y1 = 0; y2 = height; dy = 1;
            x1 = 0; x2 = width1; dx = 1;
        }
        else
        {
            y1 = height-1; y2 = -1; dy = -1;
            x1 = width1-1; x2 = -1; dx = -1;
        }

        CostType *Lr[NLR]={0}, *minLr[NLR]={0};

        for( k = 0; k < NLR; k++ )
        {
            // shift Lr[k] and minLr[k] pointers, because we allocated them with the borders,
            // and will occasionally use negative indices with the arrays
            // we need to shift Lr[k] pointers by 1, to give the space for d=-1.
            // however, then the alignment will be imperfect, i.e. bad for SSE,
            // thus we shift the pointers by 8 (8*sizeof(short) == 16 - ideal alignment)
            Lr[k] = pixDiff + costBufSize + LrSize*k + NRD2*LrBorder + 8;
            memset( Lr[k] - LrBorder*NRD2 - 8, 0, LrSize*sizeof(CostType) );
            minLr[k] = pixDiff + costBufSize + LrSize*NLR + minLrSize*k + NR2*2;
            memset( minLr[k] - LrBorder*NR2, 0, minLrSize*sizeof(CostType) );
        }

        for( int y = y1; y != y2; y += dy )
        {
            int x, d;
            DispType* disp1ptr = disp1.ptr<DispType>(y);
            CostType* C = Cbuf + (!fullDP ? 0 : y*costBufSize);
            CostType* S = Sbuf + (!fullDP ? 0 : y*costBufSize);

            if( pass == 1 ) // compute C on the first pass, and reuse it on the second pass, if any.
            {

                // costs are copied in from given grid
                for (int x = 0; x < width1; x++)
                {
                    for (int d = 0; d < D; d++)
                    {
                        // we assume that the given costs are in the range [0, 1]
                        // so we multiply them by some constant to get a reasonable
                        // resolution in the short values
                        C[x*D + d] = std::min(costThresh,costVolume(x,y,d))*10000;
                    }
                }


                // also, clear the S buffer
                for( k = 0; k < width1*D; k++ )
                    S[k] = 0;
            }

            // clear the left and the right borders
            memset( Lr[0] - NRD2*LrBorder - 8, 0, NRD2*LrBorder*sizeof(CostType) );
            memset( Lr[0] + width1*NRD2 - 8, 0, NRD2*LrBorder*sizeof(CostType) );
            memset( minLr[0] - NR2*LrBorder, 0, NR2*LrBorder*sizeof(CostType) );
            memset( minLr[0] + width1*NR2, 0, NR2*LrBorder*sizeof(CostType) );

            /*
             [formula 13 in the paper]
             compute L_r(p, d) = C(p, d) +
             min(L_r(p-r, d),
             L_r(p-r, d-1) + P1,
             L_r(p-r, d+1) + P1,
             min_k L_r(p-r, k) + P2) - min_k L_r(p-r, k)
             where p = (x,y), r is one of the directions.
             we process all the directions at once:
             0: r=(-dx, 0)
             1: r=(-1, -dy)
             2: r=(0, -dy)
             3: r=(1, -dy)
             4: r=(-2, -dy)
             5: r=(-1, -dy*2)
             6: r=(1, -dy*2)
             7: r=(2, -dy)
             */
            for( x = x1; x != x2; x += dx )
            {
                int xm = x*NR2, xd = xm*D2;

                int delta0 = minLr[0][xm - dx*NR2] + P2, delta1 = minLr[1][xm - NR2 + 1] + P2;
                int delta2 = minLr[1][xm + 2] + P2, delta3 = minLr[1][xm + NR2 + 3] + P2;

                CostType* Lr_p0 = Lr[0] + xd - dx*NRD2;
                CostType* Lr_p1 = Lr[1] + xd - NRD2 + D2;
                CostType* Lr_p2 = Lr[1] + xd + D2*2;
                CostType* Lr_p3 = Lr[1] + xd + NRD2 + D2*3;

                Lr_p0[-1] = Lr_p0[D] = Lr_p1[-1] = Lr_p1[D] =
                        Lr_p2[-1] = Lr_p2[D] = Lr_p3[-1] = Lr_p3[D] = MAX_COST;

                CostType* Lr_p = Lr[0] + xd;
                const CostType* Cp = C + x*D;
                //                cout << Cp[0] << " " << Cp[1] << endl;
                CostType* Sp = S + x*D;

#if CV_SSE2
                if( useSIMD )
                {
                    __m128i _P1 = _mm_set1_epi16((short)P1);

                    __m128i _delta0 = _mm_set1_epi16((short)delta0);
                    __m128i _delta1 = _mm_set1_epi16((short)delta1);
                    __m128i _delta2 = _mm_set1_epi16((short)delta2);
                    __m128i _delta3 = _mm_set1_epi16((short)delta3);
                    __m128i _minL0 = _mm_set1_epi16((short)MAX_COST);

                    for( d = 0; d < D; d += 8 )
                    {
                        __m128i Cpd = _mm_load_si128((const __m128i*)(Cp + d));
                        __m128i L0, L1, L2, L3;

                        L0 = _mm_load_si128((const __m128i*)(Lr_p0 + d));
                        L1 = _mm_load_si128((const __m128i*)(Lr_p1 + d));
                        L2 = _mm_load_si128((const __m128i*)(Lr_p2 + d));
                        L3 = _mm_load_si128((const __m128i*)(Lr_p3 + d));

                        L0 = _mm_min_epi16(L0, _mm_adds_epi16(_mm_loadu_si128((const __m128i*)(Lr_p0 + d - 1)), _P1));
                        L0 = _mm_min_epi16(L0, _mm_adds_epi16(_mm_loadu_si128((const __m128i*)(Lr_p0 + d + 1)), _P1));

                        L1 = _mm_min_epi16(L1, _mm_adds_epi16(_mm_loadu_si128((const __m128i*)(Lr_p1 + d - 1)), _P1));
                        L1 = _mm_min_epi16(L1, _mm_adds_epi16(_mm_loadu_si128((const __m128i*)(Lr_p1 + d + 1)), _P1));

                        L2 = _mm_min_epi16(L2, _mm_adds_epi16(_mm_loadu_si128((const __m128i*)(Lr_p2 + d - 1)), _P1));
                        L2 = _mm_min_epi16(L2, _mm_adds_epi16(_mm_loadu_si128((const __m128i*)(Lr_p2 + d + 1)), _P1));

                        L3 = _mm_min_epi16(L3, _mm_adds_epi16(_mm_loadu_si128((const __m128i*)(Lr_p3 + d - 1)), _P1));
                        L3 = _mm_min_epi16(L3, _mm_adds_epi16(_mm_loadu_si128((const __m128i*)(Lr_p3 + d + 1)), _P1));

                        L0 = _mm_min_epi16(L0, _delta0);
                        L0 = _mm_adds_epi16(_mm_subs_epi16(L0, _delta0), Cpd);

                        L1 = _mm_min_epi16(L1, _delta1);
                        L1 = _mm_adds_epi16(_mm_subs_epi16(L1, _delta1), Cpd);

                        L2 = _mm_min_epi16(L2, _delta2);
                        L2 = _mm_adds_epi16(_mm_subs_epi16(L2, _delta2), Cpd);

                        L3 = _mm_min_epi16(L3, _delta3);
                        L3 = _mm_adds_epi16(_mm_subs_epi16(L3, _delta3), Cpd);

                        _mm_store_si128( (__m128i*)(Lr_p + d), L0);
                        _mm_store_si128( (__m128i*)(Lr_p + d + D2), L1);
                        _mm_store_si128( (__m128i*)(Lr_p + d + D2*2), L2);
                        _mm_store_si128( (__m128i*)(Lr_p + d + D2*3), L3);

                        __m128i t0 = _mm_min_epi16(_mm_unpacklo_epi16(L0, L2), _mm_unpackhi_epi16(L0, L2));
                        __m128i t1 = _mm_min_epi16(_mm_unpacklo_epi16(L1, L3), _mm_unpackhi_epi16(L1, L3));
                        t0 = _mm_min_epi16(_mm_unpacklo_epi16(t0, t1), _mm_unpackhi_epi16(t0, t1));
                        _minL0 = _mm_min_epi16(_minL0, t0);

                        __m128i Sval = _mm_load_si128((const __m128i*)(Sp + d));

                        L0 = _mm_adds_epi16(L0, L1);
                        L2 = _mm_adds_epi16(L2, L3);
                        Sval = _mm_adds_epi16(Sval, L0);
                        Sval = _mm_adds_epi16(Sval, L2);

                        _mm_store_si128((__m128i*)(Sp + d), Sval);
                    }

                    _minL0 = _mm_min_epi16(_minL0, _mm_srli_si128(_minL0, 8));
                    _mm_storel_epi64((__m128i*)&minLr[0][xm], _minL0);
                }
                else
#endif
                {
                    int minL0 = MAX_COST, minL1 = MAX_COST, minL2 = MAX_COST, minL3 = MAX_COST;

                    for( d = 0; d < D; d++ )
                    {
                        int Cpd = Cp[d], L0, L1, L2, L3;

                        L0 = Cpd + std::min((int)Lr_p0[d], std::min(Lr_p0[d-1] + P1, std::min(Lr_p0[d+1] + P1, delta0))) - delta0;
                        L1 = Cpd + std::min((int)Lr_p1[d], std::min(Lr_p1[d-1] + P1, std::min(Lr_p1[d+1] + P1, delta1))) - delta1;
                        L2 = Cpd + std::min((int)Lr_p2[d], std::min(Lr_p2[d-1] + P1, std::min(Lr_p2[d+1] + P1, delta2))) - delta2;
                        L3 = Cpd + std::min((int)Lr_p3[d], std::min(Lr_p3[d-1] + P1, std::min(Lr_p3[d+1] + P1, delta3))) - delta3;

                        Lr_p[d] = (CostType)L0;
                        minL0 = std::min(minL0, L0);

                        Lr_p[d + D2] = (CostType)L1;
                        minL1 = std::min(minL1, L1);

                        Lr_p[d + D2*2] = (CostType)L2;
                        minL2 = std::min(minL2, L2);

                        Lr_p[d + D2*3] = (CostType)L3;
                        minL3 = std::min(minL3, L3);

                        Sp[d] = cv::saturate_cast<CostType>(Sp[d] + L0 + L1 + L2 + L3);
                    }
                    minLr[0][xm] = (CostType)minL0;
                    minLr[0][xm+1] = (CostType)minL1;
                    minLr[0][xm+2] = (CostType)minL2;
                    minLr[0][xm+3] = (CostType)minL3;
                }
            }

            if( pass == npasses )
            {
                for( x = 0; x < width; x++ )
                {
                    disp1ptr[x] = (DispType)INVALID_DISP_SCALED;
                }

                for( x = width1 - 1; x >= 0; x-- )
                {
                    CostType* Sp = S + x*D;
                    int minS = MAX_COST, bestDisp = -1;

                    if( npasses == 1 )
                    {
                        int xm = x*NR2, xd = xm*D2;

                        int minL0 = MAX_COST;
                        int delta0 = minLr[0][xm + NR2] + P2;
                        CostType* Lr_p0 = Lr[0] + xd + NRD2;
                        Lr_p0[-1] = Lr_p0[D] = MAX_COST;
                        CostType* Lr_p = Lr[0] + xd;

                        const CostType* Cp = C + x*D;

#if CV_SSE2
                        if( useSIMD )
                        {
                            __m128i _P1 = _mm_set1_epi16((short)P1);
                            __m128i _delta0 = _mm_set1_epi16((short)delta0);

                            __m128i _minL0 = _mm_set1_epi16((short)minL0);
                            __m128i _minS = _mm_set1_epi16(MAX_COST), _bestDisp = _mm_set1_epi16(-1);
                            __m128i _d8 = _mm_setr_epi16(0, 1, 2, 3, 4, 5, 6, 7), _8 = _mm_set1_epi16(8);

                            for( d = 0; d < D; d += 8 )
                            {
                                __m128i Cpd = _mm_load_si128((const __m128i*)(Cp + d)), L0;

                                L0 = _mm_load_si128((const __m128i*)(Lr_p0 + d));
                                L0 = _mm_min_epi16(L0, _mm_adds_epi16(_mm_loadu_si128((const __m128i*)(Lr_p0 + d - 1)), _P1));
                                L0 = _mm_min_epi16(L0, _mm_adds_epi16(_mm_loadu_si128((const __m128i*)(Lr_p0 + d + 1)), _P1));
                                L0 = _mm_min_epi16(L0, _delta0);
                                L0 = _mm_adds_epi16(_mm_subs_epi16(L0, _delta0), Cpd);

                                _mm_store_si128((__m128i*)(Lr_p + d), L0);
                                _minL0 = _mm_min_epi16(_minL0, L0);
                                L0 = _mm_adds_epi16(L0, *(__m128i*)(Sp + d));
                                _mm_store_si128((__m128i*)(Sp + d), L0);

                                __m128i mask = _mm_cmpgt_epi16(_minS, L0);
                                _minS = _mm_min_epi16(_minS, L0);
                                _bestDisp = _mm_xor_si128(_bestDisp, _mm_and_si128(_mm_xor_si128(_bestDisp,_d8), mask));
                                _d8 = _mm_adds_epi16(_d8, _8);
                            }

                            short CV_DECL_ALIGNED(16) bestDispBuf[8];
                            _mm_store_si128((__m128i*)bestDispBuf, _bestDisp);

                            _minL0 = _mm_min_epi16(_minL0, _mm_srli_si128(_minL0, 8));
                            _minL0 = _mm_min_epi16(_minL0, _mm_srli_si128(_minL0, 4));
                            _minL0 = _mm_min_epi16(_minL0, _mm_srli_si128(_minL0, 2));

                            __m128i qS = _mm_min_epi16(_minS, _mm_srli_si128(_minS, 8));
                            qS = _mm_min_epi16(qS, _mm_srli_si128(qS, 4));
                            qS = _mm_min_epi16(qS, _mm_srli_si128(qS, 2));

                            minLr[0][xm] = (CostType)_mm_cvtsi128_si32(_minL0);
                            minS = (CostType)_mm_cvtsi128_si32(qS);

                            qS = _mm_shuffle_epi32(_mm_unpacklo_epi16(qS, qS), 0);
                            qS = _mm_cmpeq_epi16(_minS, qS);
                            int idx = _mm_movemask_epi8(_mm_packs_epi16(qS, qS)) & 255;

                            bestDisp = bestDispBuf[LSBTab[idx]];
                        }
                        else
#endif
                        {
                            for( d = 0; d < D; d++ )
                            {
                                int L0 = Cp[d] + std::min((int)Lr_p0[d], std::min(Lr_p0[d-1] + P1, std::min(Lr_p0[d+1] + P1, delta0))) - delta0;

                                Lr_p[d] = (CostType)L0;
                                minL0 = std::min(minL0, L0);

                                int Sval = Sp[d] = cv::saturate_cast<CostType>(Sp[d] + L0);
                                if( Sval < minS )
                                {
                                    minS = Sval;
                                    bestDisp = d;
                                }
                            }
                            minLr[0][xm] = (CostType)minL0;
                        }
                    }
                    else
                    {
                        for( d = 0; d < D; d++ )
                        {
                            int Sval = Sp[d];
                            if( Sval < minS )
                            {
                                minS = Sval;
                                bestDisp = d;
                            }
                        }
                    }

                    if (outputUniquenessRatiosEnabled)
                    {
                        for (int d = 0; d < D; d++)
                        {
                            if (costVolume(x,y,d) == 0)
                                uniquenessRatios(x,y) = 0;
                            else if (costVolume(x,y,bestDisp)/costVolume(x,y,d) < uniquenessRatios(x,y))
                                uniquenessRatios(x,y)  = costVolume(x,y,bestDisp)/costVolume(x,y,d);
                        }
                    }

                    if (outputCostsEnabled)
                    {
                        costs(x,y) = costVolume(x,y,bestDisp);
                    }

                    if (outputDepthMapEnabled)
                    {
                        // compute depth from plane id
                        double d = planes(bestDisp,0)(3);

                        //std::cout << "D = " << D << "  bestDisp = " << bestDisp << "  subPixelEnabled = " << subPixelEnabled << std::endl;

                        if (subPixelEnabled && bestDisp > 0 && bestDisp < D-1)
                        {
                            const double denom = Sp[bestDisp+1] + Sp[bestDisp-1] - 2*Sp[bestDisp];
                           // std::cout << "denom = " << denom << std::endl;
                            double offset = 0.0;
                            if (denom > 1e-2)
                            {
                                offset = (Sp[bestDisp-1] - Sp[bestDisp])/denom - 0.5f;
                            }

                            if (offset < 0)
                            {
                                const int oPlaneIdx = bestDisp -1;

                                float oPlaneD = planes(oPlaneIdx,0)(3);

                                float dStep = (1.0f/oPlaneD - 1.0f/d)*offset;
                                //std::cout << "dStep = " << dStep << std::endl;
                                d = 1.0f/(1.0f/d - dStep);

                            }
                            if (offset > 0)
                            {
                                const int oPlaneIdx = bestDisp + 1;

                                float oPlaneD = planes(oPlaneIdx,0)(3);

                                float dStep = (1.0f/d - 1.0f/oPlaneD)*offset;
                                //std::cout << "dStep = " << dStep << std::endl;
                                d = 1.0f/(1.0f/d - dStep);
                            }
                        }
                        Matrix<double, 3, 1> n = planes(bestDisp, 0).head(3);
                        Matrix<double, 3, 1> pos;
                        pos(0) = x; pos(1) = y; pos(2) = 1;

                        double denom = pos.transpose()*refKinvT*n;
                        result(x,y) = -d/denom;
                    }
                }
            }

            // now shift the cyclic buffers
            std::swap( Lr[0], Lr[1] );
            std::swap( minLr[0], minLr[1] );
        }
    }
}

void SemiGlobalMatching::setSmoothness(int P1, int P2)
{
    if (!(P2 > P1 && P1 > 0))
    {
        D3D_THROW_EXCEPTION("P2 must be bigger than P1 and both positive.");
    }

    this->P1 = P1;
    this->P2 = P2;
}

DepthMap<float, double> SemiGlobalMatching::getDepthMap()
{
    return result;
}

Grid<int> SemiGlobalMatching::getLabels()
{
    return labels;
}

void SemiGlobalMatching::enableOutputCostsEnabled(bool enabled)
{
    this->outputCostsEnabled = enabled;
}

void SemiGlobalMatching::enableOutputDepthMap(bool enabled)
{
    this->outputDepthMapEnabled = enabled;
}

void SemiGlobalMatching::enableOutputUniquenessRatios(bool enabled)
{
    this->outputUniquenessRatiosEnabled = enabled;
}

Grid<float> SemiGlobalMatching::getCosts()
{
    return costs;
}

Grid<float> SemiGlobalMatching::getUniquenessRatios()
{
    return uniquenessRatios;
}

void SemiGlobalMatching::enableOutputLabels(bool enabled)
{
    this->outputLabelsEnabled = enabled;
}

void SemiGlobalMatching::matchFromPlaneSweepWtihImage(const Grid<float> &costVolume, const Grid<Eigen::Vector4d> &planes, const CameraMatrix<double> &cam, cv::Mat image, float P1F, float P2F, float k)
{
    Matrix<double, 3, 3> refKinvT = Matrix<double, 3, 3>::Zero();
    if (outputDepthMapEnabled)
    {
        result = DepthMap<float, double>(costVolume.getWidth(), costVolume.getHeight(), cam);
        refKinvT  = cam.getK().inverse().transpose();
    }

    regularizeCostVolumeWithImage(costVolume, image, P1F, P2F, k);

    if (outputDepthMapEnabled)
    {
        for (unsigned int y = 0; y < costVolume.getHeight(); y++)
            for (unsigned int x = 0; x < costVolume.getWidth(); x++)
            {
                Matrix<double, 3, 1> n = planes(labels(x,y), 0).head(3);
                double d = planes(labels(x,y),0)(3);

                if (subPixelEnabled && labels(x,y) > 0 && labels(x,y) < (int)costVolume.getHeight()-1)
                {

                    if (offsets(x,y) < 0)
                    {
                        const int oPlaneIdx = labels(x,y) -1;

                        float oPlaneD = planes(oPlaneIdx,0)(3);

                        float dStep = (1.0f/oPlaneD - 1.0f/d)*offsets(x,y);
                        //std::cout << "dStep = " << dStep << std::endl;
                        d = 1.0f/(1.0f/d - dStep);

                    }
                    if (offsets(x,y) > 0)
                    {
                        const int oPlaneIdx = labels(x,y) + 1;

                        float oPlaneD = planes(oPlaneIdx,0)(3);

                        float dStep = (1.0f/d - 1.0f/oPlaneD)*offsets(x,y);
                        //std::cout << "dStep = " << dStep << std::endl;
                        d = 1.0f/(1.0f/d - dStep);
                    }
                }

                Matrix<double, 3, 1> pos;
                pos(0) = x; pos(1) = y; pos(2) = 1;

                double denom = pos.transpose()*refKinvT*n;
                result(x,y) = -d/denom;
            }
    }


}


// Non optimized but easy to change version of semi global matching
void SemiGlobalMatching::regularizeCostVolumeWithImage(const Grid<float>& costVolume, cv::Mat image, float P1F, float P2F, float k)
{
    D3D::Grid<float> summedCosts = D3D::Grid<float>(costVolume.getWidth(), costVolume.getHeight(), costVolume.getDepth());
    const int numPaths = 8;

    D3D::Grid<float> pathCosts = D3D::Grid<float>(costVolume.getWidth(), costVolume.getHeight(), costVolume.getDepth());

    int r1, r2;
    int xS, yS;
    int xR, yR;
    int xE, yE;

    for (int i = 0; i < numPaths; i++)
    {
        switch (i)
        {
        case 0:
            r1 = -1;
            r2 = 1;
            xS = costVolume.getWidth()-1;
            yS = 0;
            xR = -1;
            yR = 1;
            xE = -1;
            yE = costVolume.getHeight();
            break;
        case 1:
            r1 = 1;
            r2 = -1;
            xS = 0;
            yS = costVolume.getHeight()-1;
            xR = 1;
            yR = -1;
            xE = costVolume.getWidth();
            yE = -1;
            break;
        case 2:
            r1 = -1;
            r2 = -1;
            xS = costVolume.getWidth()-1;
            yS = costVolume.getHeight()-1;
            xR = -1;
            yR = -1;
            xE = -1;
            yE = -1;
            break;
        case 3:
            r1 = 1;
            r2 = 1;
            xS = 0;
            yS = 0;
            xR = 1;
            yR = 1;
            xE = costVolume.getWidth();
            yE = costVolume.getHeight();
            break;
        case 4:
            r1 = 0;
            r2 = -1;
            xS = 0;
            yS = costVolume.getHeight() - 1;
            xR = 1;
            yR = -1;
            xE = costVolume.getWidth();
            yE = -1;
            break;
        case 5:
            r1 = 0;
            r2 = 1;
            xS = 0;
            yS = 0;
            xR = 1;
            yR = 1;
            xE = costVolume.getWidth();
            yE = costVolume.getHeight();
            break;
        case 6:
            r1 = -1;
            r2 = 0;
            xS = costVolume.getWidth() - 1;
            yS = 0;
            xR = -1;
            yR = 1;
            xE = -1;
            yE = costVolume.getHeight();
            break;
        case 7:
            r1 = 1;
            r2 = 0;
            xS = 0;
            yS = 0;
            xR = 1;
            yR = 1;
            xE = costVolume.getWidth();
            yE = costVolume.getHeight();
            break;
        }

        for (int y = yS; y != yE; y += yR)
        {
            std::cout << y << std::endl;
            for (int x = xS; x != xE; x += xR)
            {
                // std::cout << xE << std::endl;
                // std::cout << x << std::endl;

                const int x0 = x - r1;
                const int y0 = y - r2;

                if (x0 < 0 || y0 < 0 || x0 >= (int) costVolume.getWidth() || y0 >= (int) costVolume.getHeight())
                {
                    for (int d = 0; d < (int) costVolume.getDepth(); d++)
                        pathCosts(x,y,d) = costVolume(x,y,d);
                }
                else
                {
                    // compute P2' based on the image gradient
//                    const float ib0 = (float) image.at<unsigned char>(y0,3*x0)/255.0f;
//                    const float ib1 = (float) image.at<unsigned char>(y,3*x)/255.0f;
//                    const float ig0 = (float) image.at<unsigned char>(y0,3*x0+1)/255.0f;
//                    const float ig1 = (float) image.at<unsigned char>(y,3*x+1)/255.0f;
//                    const float ir0 = (float) image.at<unsigned char>(y0,3*x0+2)/255.0f;
//                    const float ir1 = (float) image.at<unsigned char>(y,3*x+2)/255.0f;

//                    const float grad = std::fabs(ib0 - ib1) + std::fabs(ig0 - ig1) + std::fabs(ir0 - ir1);
                    const float P2Fp = P2F; //std::max(P1F, k*P2F*3/grad);

                    float lastMin = pathCosts(x0,y0,0);
                    for (int d = 1; d < (int) costVolume.getDepth(); d++)
                    {
                        if (pathCosts(x0,y0,d) < lastMin)
                            lastMin = pathCosts(x0,y0,d);
                    }

                    const float jumpCost = lastMin + P2Fp;

                    for (int d = 0; d < (int) costVolume.getDepth(); d++)
                    {
                        float dispMinCost = std::min(jumpCost, pathCosts(x0,y0,d));
                        if (d-1 >= 0)
                        {
                            const float m1Cost = pathCosts(x0,y0,d-1) + P1F;
                            if (m1Cost < dispMinCost)
                                dispMinCost = m1Cost;
                        }
                        if (d+1 < (int) costVolume.getDepth())
                        {
                            const float p1Cost = pathCosts(x0,y0,d+1) + P1F;
                            if (p1Cost < dispMinCost)
                                dispMinCost = p1Cost;
                        }

                        pathCosts(x,y,d) = costVolume(x,y,d) + dispMinCost - lastMin;
                    }
                }
            }
        }

        for (int y = 0; y < (int) costVolume.getHeight(); y++)
            for (int x = 0; x < (int) costVolume.getWidth(); x++)
                for (int d = 0; d < (int) costVolume.getDepth(); d++)
                {
                    if (i == 0)
                    {
                        summedCosts(x,y,d) = pathCosts(x,y,d)/(float)numPaths;
                    }
                    else
                    {
                        summedCosts(x,y,d) += pathCosts(x,y,d)/(float)numPaths;
                    }
                }
    }

    pathCosts.resize(1,1,1);

    labels.resize(costVolume.getWidth(), costVolume.getHeight(), 1);

    if (subPixelEnabled)
    {
        offsets.resize(costVolume.getWidth(), costVolume.getHeight(), 1);
    }

    for (int y = 0; y < (int) costVolume.getHeight(); y++)
        for (int x = 0; x < (int) costVolume.getWidth(); x++)
        {
            int bestLabel = 0;
            float bestCost = summedCosts(x,y,0);

            for (int d = 1; d < (int) costVolume.getDepth(); d++)
            {
                if (summedCosts(x,y,d) < bestCost)
                {
                    bestLabel = d;
                    bestCost = summedCosts(x,y,d);
                }
            }

            if (subPixelEnabled)
            {
                // if subpixel is enabled we need to compute the label offsets
                offsets(x,y) = 0;

                if (bestLabel > 0 && bestLabel < (int)costVolume.getDepth()-1)
                {
                    const float denom = summedCosts(x,y,bestLabel+1) + summedCosts(x,y,bestLabel-1) - 2*summedCosts(x,y,bestLabel);
                    // std::cout << "denom = " << denom << std::endl;
                    if (denom > 1e-2)
                    {
                        offsets(x,y) = (summedCosts(x,y,bestLabel-1) - summedCosts(x,y,bestLabel))/denom - 0.5f;
                    }
                }

            }

            labels(x,y) = bestLabel;
        }
}
