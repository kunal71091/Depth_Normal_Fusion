#include "stereoTools.h"
#include <iostream>

using D3D::Grid;

Grid<double> D3D::computeAD(Mat img1, Mat img2, int minDisp, int maxDisp)
{
    int numDisps = maxDisp-minDisp+1;
    int numChannels = img1.channels();
    double accFact = 1/img1.channels();

    Grid<double> ADs(img1.cols, img1.rows, numDisps);

    int dIdx = 0;
    for (int d = minDisp; d <= maxDisp; d++)
    {
        for (int y = 0; y < img1.rows; y++)
        {
            for (int x1 = 0; x1 < img1.cols; x1++)
            {
                int x2 = x1-d;
                if (x2 < 0)
                    x2 = 0;
                if (x2 >= img2.cols)
                    x2 = img2.cols-1;

                for (int c = 0; c < img1.channels(); c++)
                {
                    ADs(x1,y,dIdx) += accFact*(std::abs((double)img1.at<unsigned char>(y,numChannels*x1) - (double)img2.at<unsigned char>(y,numChannels*x2))/(double)255);

                }
            }
        }
        dIdx++;
    }

    return ADs;
}

D3D::Grid<float> D3D::computeBTLeft(cv::Mat imgL, cv::Mat imgR, int minDisp, int maxDisp, int windowR)
{
    int numDisps = maxDisp-minDisp+1;
    int numChannels = imgL.channels();
    float accFact = 1.0f/numChannels;

    Grid<float> BTs(imgL.cols, imgR.rows, numDisps);

    D3D::Grid<float> ILmin(numChannels, imgL.cols);
    D3D::Grid<float> ILmax(numChannels, imgL.cols);
    D3D::Grid<float> IRmin(numChannels, imgR.cols);
    D3D::Grid<float> IRmax(numChannels, imgR.cols);

    for (int y = 0; y < imgL.rows; y++)
    {
        // compute I minus and I plus
        for (int x = 0; x < imgL.cols; x++)
        {
            int xM = (x-1 < 0) ? imgL.cols-1 : x-1;
            int xP = (x+1 == imgL.cols) ? 0 : x+1;

            for (int c = 0; c < numChannels; c++)
            {

                const float ILm = 0.5f*(imgL.at<unsigned char>(y, numChannels*x + c) + imgL.at<unsigned char>(y, numChannels*xM + c));
                const float ILp = 0.5f*(imgL.at<unsigned char>(y, numChannels*x + c) + imgL.at<unsigned char>(y, numChannels*xP + c));
                const float IL = imgL.at<unsigned char>(y, numChannels*x + c);
                ILmin(c,x) = std::min(std::min(ILm,ILp), IL);
                ILmax(c,x) = std::max(std::max(ILm,ILp), IL);

                const float IRm = 0.5f*(imgR.at<unsigned char>(y, numChannels*x + c) + imgR.at<unsigned char>(y, numChannels*xM + c));
                const float IRp = 0.5f*(imgR.at<unsigned char>(y, numChannels*x + c) + imgR.at<unsigned char>(y, numChannels*xP + c));
                const float IR = imgR.at<unsigned char>(y, numChannels*x + c);
                IRmin(c,x) = std::min(std::min(IRm, IRp), IR);
                IRmax(c,x) = std::max(std::max(IRm, IRp), IR);
            }

            // compute the actual costs
            float bestBT = 255;
            for (int d = 0; d < numDisps; d++)
            {
                const int disp = d + minDisp;
                if (x - disp >= 0)
                {
                    int xR = x - disp;

                    BTs(x,y,d) = 0;

                    for (int c = 0; c < numChannels; c++)
                    {
                        const float dLR = std::max(std::max(0.0f,imgL.at<unsigned char>(y, numChannels*x + c) - IRmax(c,xR)), IRmin(c,xR) - imgL.at<unsigned char>(y, numChannels*x + c));
                        const float dRL = std::max(std::max(0.0f,imgR.at<unsigned char>(y, numChannels*xR + c) - ILmax(c,x)), ILmin(c,x) - imgR.at<unsigned char>(y, numChannels*xR + c));

                        BTs(x,y,d) += accFact*std::min(dLR, dRL);
                    }

                   if (BTs(x,y,d) < bestBT)
                   {
                       bestBT = BTs(x,y,d);
                   }
                }
                else
                {
                        BTs(x,y,d) = bestBT + std::min(std::abs(x - disp), 5);
                }
            }
        }
    }
    
    D3D::Grid<float> datacost(BTs.getWidth(), BTs.getHeight(), BTs.getDepth());
    
        // sum up windows
    for (unsigned int z = 0; z < BTs.getDepth(); z++)
        for (unsigned int y = 0; y < BTs.getHeight(); y++)
            for (unsigned int x = 0; x < BTs.getWidth(); x++)
            {
                float datacostSum = 0;
                int numVal = 0;
                for (int wx = -windowR; wx <= windowR; wx++)
                    for (int wy = -windowR; wy <= windowR; wy++)
                    {
                        const int X = (int) x + (int) wx;
                        const int Y = (int) y + (int) wy;

                        if (X >= 0 && X < (int) datacost.getWidth() && Y >= 0 && Y < (int) datacost.getHeight())
                        {
                            numVal++;
                            float bestBTInregion = BTs(X,Y,z);
//                            for (int wz = -windowR; wz <= windowR; wz++)
//                            {
//                                const int Z = z + wz;
//                                if (Z >= 0 && Z < (int) BTs.getDepth())
//                                {
//                                    if (wx != 0 && wy != 0 && BTs(X,Y,Z) < bestBTInregion)
//                                    {
//                                        bestBTInregion = BTs(X,Y,Z);
//                                    }
//                                }
//                            }

                            datacostSum += bestBTInregion;
                        }
                    }

                if (numVal > 0)
                {
                    datacost(x,y,z) = datacostSum/numVal;
                }
                else
                {
                    datacost(x,y,z) = 0;
                }
            }

    return datacost;
}

D3D::Grid<float> D3D::computeCensusMatchLeft(cv::Mat imgL, cv::Mat imgR, int minDisp, int maxDisp, int windowR)
{  
  int numDisps = maxDisp-minDisp+1;
  int numChannels = imgL.channels();
  
  // census transform both of the images
  D3D::Grid<unsigned char> censusLeft(imgL.cols, imgL.rows, numChannels, 0);
  D3D::Grid<unsigned char> censusRight(imgR.cols, imgR.rows, numChannels, 0);
  
  for (unsigned int y = 0; y < censusLeft.getHeight(); y++)
    for (unsigned int x = 0; x < censusLeft.getWidth(); x++)
      for (int c = 0; c < numChannels; c++)
      {
	int i = 0;
	for (int xw = -1; xw <= 1; xw++)
	  for (int yw = -1; yw <= 1; yw++)
	  {
	    if (xw != (int) x && yw != (int) y)
	    {
	      const int X = x + xw;
	      const int Y = y + yw;
	      
	      if (X >= 0 && X < (int) censusLeft.getWidth() && Y >= 0 && Y < (int) censusLeft.getHeight())
	      {
        if (imgL.at<unsigned char>(y,numChannels*x) > imgL.at<unsigned char>(Y,numChannels*X))
		{
		  censusLeft(x,y,c) |= 1 << i;
		}
		else
		{
		  censusLeft(x,y,c) &= ~(1 << i);
		}
		
        if (imgR.at<unsigned char>(y,numChannels*x) > imgR.at<unsigned char>(Y,numChannels*X))
		{
		  censusRight(x,y,c) |= 1 << i;
		}
		else
		{
		  censusRight(x,y,c) &= ~(1 << i);
		}
	      }
	      else
	      {
		censusLeft(x,y,c) &= ~(1 << i);
		censusRight(x,y,c) &= ~(1 << i);
	      }
	      
	      i++;
	    }
	  }
      }
      
  // census transform matching
  D3D::Grid<float> censusMatch(censusLeft.getWidth(), censusRight.getHeight(), numDisps, 0);

  for (int y = 0; y < imgL.rows; y++)
  {
    for (int x = 0; x < imgL.cols; x++)
    { 
      int dIdx = 0;
      
      float bestCensusScore = numChannels*8;
      int bestCensusDisp = minDisp;
      for (int d = minDisp; d <= maxDisp; d++)
      {
	int xR = x-d;
	
	if (xR > 0 && xR < (int) censusLeft.getWidth())
	{
	  for (int c = 0; c < numChannels; c++)
	  {
	    unsigned char Xor = censusLeft(x,y,c) ^ censusRight(xR,y,c);
        censusMatch(x,y,dIdx) += 1.5*(float) __builtin_popcount(Xor);
	  }
	  
      if (censusMatch(x,y,dIdx) < bestCensusScore)
	  {
        bestCensusScore = censusMatch(x,y,dIdx);
        bestCensusDisp = d;
	  }
	}
	else
	{
	  if (d - bestCensusDisp < 5 && bestCensusDisp - d > 0)
	  {
	    censusMatch(x,y,dIdx) = censusMatch(x,y,bestCensusDisp -d)+3.0f;
	  }
	  else
	  {
	    censusMatch(x,y,dIdx) = bestCensusScore +3.0f;
	  }
	}
        
        dIdx++;
      }
    }
  }

  D3D::Grid<float> datacost(censusMatch.getWidth(), censusMatch.getHeight(), censusMatch.getDepth());

  // sum up windows
  for (unsigned int z = 0; z < censusMatch.getDepth(); z++)
      for (unsigned int y = 0; y < censusMatch.getHeight(); y++)
          for (unsigned int x = 0; x < censusMatch.getWidth(); x++)
          {
              float datacostSum = 0;
              int numVal = 0;
              for (int wx = -windowR; wx <= windowR; wx++)
                  for (int wy = -windowR; wy <= windowR; wy++)
                  {
                      const int X = (int) x + (int) wx;
                      const int Y = (int) y + (int) wy;

                      if (X >= 0 && X < (int) datacost.getWidth() && Y >= 0 && Y < (int) datacost.getHeight())
                      {
                          numVal++;
                          float bestCensusMatchInregion = censusMatch(X,Y,z);
//                          for (int wz = -windowR; wz <= windowR; wz++)
//                          {
//                              const int Z = z + wz;
//                              if (Z >= 0 && Z < (int) censusMatch.getDepth())
//                              {
//                                  if (wx != 0 && wy != 0 && censusMatch(X,Y,Z) < bestCensusMatchInregion)
//                                  {
//                                      bestCensusMatchInregion = censusMatch(X,Y,Z);
//                                  }
//                              }
//                          }

                          datacostSum += bestCensusMatchInregion;
                      }
                  }

              if (numVal > 0)
              {
                  datacost(x,y,z) = datacostSum/numVal;
              }
              else
              {
                  datacost(x,y,z) = 0;
              }
          }

  return datacost;
}


D3D::Grid<float> D3D::computeSobelMatchLeft(cv::Mat imgL, cv::Mat imgR, int minDisp, int maxDisp, int windowR)
{
  int numDisps = maxDisp-minDisp+1;
  int numChannels = imgL.channels();

  // census transform both of the images
  D3D::Grid<float> sobelXLeft(imgL.cols, imgL.rows, numChannels, 0);
  D3D::Grid<float> sobelYLeft(imgL.cols, imgL.rows, numChannels, 0);
  D3D::Grid<float> sobelXRight(imgL.cols, imgL.rows, numChannels, 0);
  D3D::Grid<float> sobelYRight(imgL.cols, imgL.rows, numChannels, 0);

  Eigen::Matrix3f SobelX;
  SobelX(0,0) = -1; SobelX(0,1) = 0; SobelX(0,2) = 1;
  SobelX(1,0) = -2; SobelX(1,1) = 0; SobelX(1,2) = 2;
  SobelX(2,0) = -1; SobelX(2,1) = 0; SobelX(2,2) = 1;

  Eigen::Matrix3f SobelY = SobelX.transpose();

  for (unsigned int y = 0; y < sobelXLeft.getHeight(); y++)
      for (unsigned int x = 0; x < sobelXLeft.getWidth(); x++)
          for (int c = 0; c < numChannels; c++)
          {
              for (int xw = -1; xw <= 1; xw++)
                  for (int yw = -1; yw <= 1; yw++)
                  {
                      const int X = x + xw;
                      const int Y = y + yw;

                      if (X >= 0 && X < (int) sobelXLeft.getWidth() && Y >= 0 && Y < (int) sobelXLeft.getHeight())
                      {
                          sobelXLeft(x,y,c) += SobelX(xw+1,yw+1)*imgL.at<unsigned char>(y,numChannels*X);
                          sobelYLeft(x,y,c) += SobelY(xw+1,yw+1)*imgL.at<unsigned char>(y,numChannels*X);

                          sobelXRight(x,y,c) += SobelX(xw+1,yw+1)*imgR.at<unsigned char>(y,numChannels*X);
                          sobelYRight(x,y,c) += SobelY(xw+1,yw+1)*imgR.at<unsigned char>(y,numChannels*X);
                      }
                  }
          }

  // census transform matching
  D3D::Grid<float> sobelMatch(sobelXLeft.getWidth(), sobelXLeft.getHeight(), numDisps, 0);

  for (int y = 0; y < imgL.rows; y++)
  {
      for (int x = 0; x < imgL.cols; x++)
      {
          int dIdx = 0;

          float bestSobelScore = 1e10;
	  int bestScoreDisp = minDisp;
          for (int d = minDisp; d <= maxDisp; d++)
          {
              int xR = x-d;

              if (xR > 0 && xR < (int) sobelXLeft.getWidth())
              {
                  for (int c = 0; c < numChannels; c++)
                  {
                      sobelMatch(x,y,dIdx) += 1/16.0f*(std::fabs(sobelXLeft(x,y) - sobelXRight(xR,y)) + std::fabs(sobelYLeft(x,y) - sobelYRight(xR,y)));
                  }

                  if (sobelMatch(x,y,dIdx) < bestSobelScore)
                  {
                      bestSobelScore = sobelMatch(x,y,dIdx);
                      bestScoreDisp = d;
                  }
              }
              else
              {
		if (d - bestScoreDisp < 5 && bestScoreDisp - d > 0)
		{
                  sobelMatch(x,y,dIdx) = sobelMatch(x,y,bestScoreDisp-d)+3.0f;
		}
		else
		{
		  sobelMatch(x,y,dIdx) = bestSobelScore+3.0f;
		}
              }

              dIdx++;
          }
      }
  }

  D3D::Grid<float> datacost(sobelMatch.getWidth(), sobelMatch.getHeight(), sobelMatch.getDepth());

  // sum up windows
  for (unsigned int z = 0; z < sobelMatch.getDepth(); z++)
      for (unsigned int y = 0; y < sobelMatch.getHeight(); y++)
          for (unsigned int x = 0; x < sobelMatch.getWidth(); x++)
          {
              float datacostSum = 0;
              int numVal = 0;
              for (int wx = -windowR; wx <= windowR; wx++)
                  for (int wy = -windowR; wy <= windowR; wy++)
                  {
                      const int X = (int) x + (int) wx;
                      const int Y = (int) y + (int) wy;

                      if (X >= 0 && X < (int) datacost.getWidth() && Y >= 0 && Y < (int) datacost.getHeight())
                      {
                          numVal++;
                          float bestSobelMatchInregion = sobelMatch(X,Y,z);
//                          for (int wz = -windowR; wz <= windowR; wz++)
//                          {
//                              const int Z = z + wz;
//                              if (Z >= 0 && Z < (int) sobelMatch.getDepth())
//                              {
//                                  if (wx != 0 && wy != 0 && sobelMatch(X,Y,Z) < bestSobelMatchInregion)
//                                  {
//                                      bestSobelMatchInregion = sobelMatch(X,Y,Z);
//                                  }
//                              }
//                          }

                          datacostSum += bestSobelMatchInregion;
                      }
                  }

              if (numVal > 0)
              {
                  datacost(x,y,z) = datacostSum/numVal;
              }
              else
              {
                  datacost(x,y,z) = 0;
              }
          }

  return datacost;
}
