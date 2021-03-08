#include "DartBoard.h"

#include <opencv2/imgproc.hpp>

#include "PointHelper.h"
#include "Resources.h"

using namespace PointHelper;

DartBoard::DartBoard(std::list<DartArea> greenContours, std::list<DartArea> redContours, const cv::Mat& refImage)
  : greenContours(greenContours), redContours(redContours)
{
  std::list<DartArea*> dartBoardRed;
  std::list<DartArea*> dartBoardGreen;

  for (DartArea& redArea : this->redContours)
  {
    const bool gotBothNeighbours = getNeighbours(redArea, this->greenContours);

    // Confirms that the red area has neighbours that are not the same
    if (gotBothNeighbours && redArea.neighbours[0] != redArea.neighbours[1])
    {
      dartBoardRed.push_back(&redArea);
    }
  }

  for (DartArea* redArea : dartBoardRed)
  {
    redArea->red = true;
    // Adds green contours to list if they're not already in it
    for (DartArea* greenNeighbour : redArea->neighbours)
    {
      greenNeighbour->red = false;
      if (std::find(dartBoardGreen.begin(), dartBoardGreen.end(), greenNeighbour) != dartBoardGreen.end())
      {
        greenNeighbour->neighbours[1] = redArea;
      }
      else
      {
        greenNeighbour->neighbours[0] = redArea;
        dartBoardGreen.push_back(greenNeighbour);
      }
    }
  }

  for (DartArea* greenArea : dartBoardGreen)
  {
    if (greenArea->neighbours[0] == nullptr || greenArea->neighbours[1] == nullptr)
    {
      std::cout << "Something went wrong with the green areas\n";
      return;
    }
  }

  for (DartArea* redArea : dartBoardRed)
  {
    if (redArea->neighbours[0] == nullptr || redArea->neighbours[1] == nullptr)
    {
      std::cout << "Something went wrong with the red areas\n";
      return;
    }
  }

  if (dartBoardRed.size() > 20)
  {
    filterTheOddOneOut(dartBoardRed);

    if (dartBoardRed.size() != 20)
    {
      std::cout << "Couldn't properly generate red part of darboard, should have 20 but has " << dartBoardRed.size() << " areas!" << std::endl;
      return;
    }
  }
  if (dartBoardGreen.size() > 20)
  {
    filterTheOddOneOut(dartBoardGreen);

    if (dartBoardGreen.size() != 20)
    {
      std::cout << "Couldn't properly generate green part of darboard, should have 20 but has " << dartBoardGreen.size() << " areas!" << std::endl;
      return;
    }
  }
  if (dartBoardRed.size() < 20 || dartBoardGreen.size() < 20)
  {
    std::cout << "Couldn't properly generate darboard, should have 40 but has " << dartBoardRed.size() + dartBoardRed.size() << " areas!" << std::endl;
    return;
  }

  // Sort to doubles / triples
  doubles = sortAreas(dartBoardRed.back());
  std::list<DartArea*> innerCandidates;
  for (DartArea* dartArea : dartBoardRed)
  {
    if (!(std::find(doubles.begin(), doubles.end(), dartArea) != doubles.end()))
    {
      innerCandidates.push_back(dartArea);
    }
  }
  triples = sortAreas(innerCandidates.back());

  setBullseye();

  setCorners();

  setSortedMeanCorners(doubles);

  setSortedMeanCorners(triples);

  setSingles();

  setNames();

  setTitlePoint();

  ready = true;
}

bool DartBoard::getNeighbours(DartArea& areaCmp, std::list<DartArea>& areaList)
{
  const int maxDistance0 = getDistance(areaCmp.significantPoints[0], areaCmp.meanPoint);
  const int maxDistance1 = getDistance(areaCmp.significantPoints[1], areaCmp.meanPoint);

  for (DartArea& areaIter : areaList)
  {
    if (areaCmp.neighbours[0] == nullptr)
    {
      checkNeighbour(areaCmp, areaIter, 0, 0, maxDistance0);
      checkNeighbour(areaCmp, areaIter, 0, 1, maxDistance0);
    }
    if (areaCmp.neighbours[1] == nullptr)
    {
      checkNeighbour(areaCmp, areaIter, 1, 0, maxDistance1);
      checkNeighbour(areaCmp, areaIter, 1, 1, maxDistance1);
    }
    if (areaCmp.neighbours[0] != nullptr && areaCmp.neighbours[1] != nullptr)
    {
      // When both neighbours were found
      return true;
    }
  }
  return false;
}

void DartBoard::checkNeighbour(DartArea& area1, DartArea& area2, const int idxArea1, const int idxArea2, const int maxDistance)
{
  if (area1.neighbours[!idxArea1] == &area2)
  {
    return;
  }

  // Firstly looks for near point, secondly looks that second point of found area is not too near
  if (getDistance(area1.significantPoints[idxArea1], area2.significantPoints[idxArea2]) < maxDistance
    && getDistance(area1.significantPoints[idxArea1], area2.significantPoints[!idxArea2]) > maxDistance)
  {
    area1.neighbours[idxArea1] = &area2;
    // area2 connecting removed, because green contours can become wrongly connected with this method
  }
}

void DartBoard::filterTheOddOneOut(std::list<DartArea*>& dartBoardRed)
{
  std::list<DartArea*> dartBoardTMp;

  for (DartArea* area : dartBoardRed)
  {
    bool add = true;
    for (DartArea* areaPtr : area->neighbours)
    {
      if (areaPtr->neighbours[0] == nullptr || areaPtr->neighbours[1] == nullptr)
      {
        add = false;
      }
    }
    if (add)
    {
      dartBoardTMp.push_back(area);
    }
  }
  dartBoardRed = dartBoardTMp;
}

DartAreaArray DartBoard::sortAreas(DartArea* highestYArea)
{
  DartAreaArray sorted;

  // Hardcoded, as long as dartboard is ca. upright this will suffice
  // If needed, implement algorithm that sorts from highest to lowest.
  sorted[19] = highestYArea;

  // This algorithm will be applicable to all other dartareas with enum % 20
  auto* tmp = sorted[19]->neighbours;
  for (int i = 0; i < 19; i++) // <19, because we already got the ((last)) one
  {
    if (i <= 3)
    {
      sorted[i] = tmp[0]->meanPoint.x > tmp[1]->meanPoint.x ? tmp[0] : tmp[1];
    }
    else if (i <= 8)
    {
      // y points are ordered from top(0) to bottom(max)
      sorted[i] = tmp[0]->meanPoint.y > tmp[1]->meanPoint.y ? tmp[0] : tmp[1];
    }
    else if (i <= 13)
    {
      sorted[i] = tmp[0]->meanPoint.x < tmp[1]->meanPoint.x ? tmp[0] : tmp[1];
    }
    else
    {
      sorted[i] = tmp[0]->meanPoint.y < tmp[1]->meanPoint.y ? tmp[0] : tmp[1];
    }
    tmp = sorted[i]->neighbours;
  }
  return sorted;
}

void DartBoard::setBullseye()
{
  const auto centerX = (doubles[AREA_11]->meanPoint.x + doubles[AREA_6]->meanPoint.x) / 2;
  const auto centerY = (doubles[AREA_20]->meanPoint.y + doubles[AREA_3]->meanPoint.y) / 2;
  const cv::Point centerPoint(centerX, centerY);
  
  auto nearestGreenDistance = INT16_MAX;
  DartArea nearestGreen;
  for (const DartArea area : greenContours)
  {
    const auto distance = getDistance(centerPoint, area.meanPoint);
    if (distance < nearestGreenDistance)
    {
      nearestGreenDistance = distance;
      nearestGreen = area;
    }
  }
  nearestGreen.red = false;
  singleBull = nearestGreen;

  auto nearestRedDistance = INT16_MAX;
  DartArea nearestRed;
  for (const DartArea area : redContours)
  {
    const auto distance = getDistance(centerPoint, area.meanPoint);
    if (nearestRedDistance > distance)
    {
      nearestRedDistance = distance;
      nearestRed = area;
    }
  }
  bullseye = nearestRed;

  // Get Center of outer bullseye
  auto x = 0;
  auto y = 0;
  for (const auto pt : singleBull.contour)
  {
    x += pt.x;
    y += pt.y;
  }
  const size_t outerBullseyeSize = singleBull.contour.size();
  singleBullCenter = cv::Point(x / outerBullseyeSize, y / outerBullseyeSize);

  // Get Center of inner bullseye
  x = 0;
  y = 0;
  for (const auto pt : bullseye.contour)
  {
    x += pt.x;
    y += pt.y;
  }
  const size_t innerBullseyeSize = bullseye.contour.size();
  bullseyeCenter = cv::Point(x / innerBullseyeSize, y / innerBullseyeSize);

  // Get mean radius of outer bullseye
  int radiusSum = 0;
  for (const auto pt : singleBull.contour)
  {
    radiusSum += getDistance(pt, singleBullCenter);
  }
  outerBullseyeMeanRadius = static_cast<float>(radiusSum) / outerBullseyeSize;

  // Get mean radius of inner bullseye
  radiusSum = 0;
  for (const auto pt : bullseye.contour)
  {
    radiusSum += getDistance(pt, bullseyeCenter);
  }
  innerBullseyeMeanRadius = static_cast<float>(radiusSum) / innerBullseyeSize;

  // If triples are detected as bullseyes, a simple check could be build in here
}

void DartBoard::setCorners()
{
  for (auto i = 0; i < 20; i++)
  {
    DartArea* tripleArea = triples[i];
    DartArea* doubleArea = doubles[i];

    cv::Point ptTmp1;
    cv::Point ptTmp2;

    const cv::Point centerPoint = (tripleArea->meanPoint + doubleArea->meanPoint) / 2;
    const cv::Point centerPointShiftedToDouble = (tripleArea->meanPoint + 2 * doubleArea->meanPoint) / 3;

    auto maxDistanceSig0 = getDistance(tripleArea->significantPoints[0], tripleArea->meanPoint);
    auto maxDistanceSig1 = getDistance(tripleArea->significantPoints[1], tripleArea->meanPoint);
    auto maxDist1 = 0, maxDist2 = 0;

    // Gets points that are furthest away from centerPoint on both sides (both have to be in a radius opposite to each other)
    for (const auto pt : tripleArea->contour)
    {
      auto distTmp = getDistance(pt, centerPoint);
      if (getDistance(pt, tripleArea->significantPoints[0]) < maxDistanceSig0 && maxDist1 < distTmp)
      {
        maxDist1 = distTmp;
        ptTmp1 = pt;
      }
      distTmp = getDistance(pt, centerPoint);
      if (getDistance(pt, tripleArea->significantPoints[1]) < maxDistanceSig1 && maxDist2 < distTmp)
      {
        maxDist2 = distTmp;
        ptTmp2 = pt;
      }
    }
    tripleArea->corners[DartArea::Inner1] = ptTmp1;
    tripleArea->corners[DartArea::Inner2] = ptTmp2;

    maxDistanceSig0 = getDistance(doubleArea->significantPoints[0], doubleArea->meanPoint);
    maxDistanceSig1 = getDistance(doubleArea->significantPoints[1], doubleArea->meanPoint);
    maxDist1 = 0, maxDist2 = 0;

    // Gets points that are furthest away from centerPoint on both sides (both have to be in a radius opposite to each other)
    for (const auto pt : doubleArea->contour)
    {
      auto distTmp = getDistance(pt, centerPointShiftedToDouble);
      if (getDistance(pt, doubleArea->significantPoints[0]) < maxDistanceSig0 && maxDist1 < distTmp)
      {
        maxDist1 = distTmp;
        ptTmp1 = pt;
      }
      distTmp = getDistance(pt, centerPointShiftedToDouble);
      if (getDistance(pt, doubleArea->significantPoints[1]) < maxDistanceSig1 && maxDist2 < distTmp)
      {
        maxDist2 = distTmp;
        ptTmp2 = pt;
      }
    }
    doubleArea->corners[DartArea::Outer1] = ptTmp1;
    doubleArea->corners[DartArea::Outer2] = ptTmp2;


    // Gets points which are furthest away from Outer1 and Outer2
    auto tripleCorners1_n_2 = findPoints(tripleArea->contour,
      std::array<cv::Point, 2>{tripleArea->corners[DartArea::Inner1], tripleArea->corners[DartArea::Inner2]}, false);
    // Reversed order because Inner1 and Outer1 should be on the same side, but the algorithm alway gets the diagonally furthest away
    tripleArea->corners[DartArea::Outer2] = tripleCorners1_n_2[0];
    tripleArea->corners[DartArea::Outer1] = tripleCorners1_n_2[1];

    // Gets points which are furthest away from the outerPoint (mean of both other corners)
    // & are further away than minDistance from corners
    // & are within a radius of the corners (so that 2 points are achieved, not just 2 next to each other)
    const int minDistance = (getDistance(tripleArea->corners[DartArea::Inner1], tripleArea->corners[DartArea::Outer1])
      + getDistance(tripleArea->corners[DartArea::Inner2], tripleArea->corners[DartArea::Outer2])) / 6;
    maxDistanceSig0 = getDistance(doubleArea->significantPoints[0], doubleArea->meanPoint);
    maxDistanceSig1 = getDistance(doubleArea->significantPoints[1], doubleArea->meanPoint);
    const cv::Point outerPoint = (doubleArea->corners[DartArea::Outer1] + doubleArea->corners[DartArea::Outer2]) / 2;
    maxDist1 = maxDist2 = 0;
    for (const auto pt : doubleArea->contour)
    {
      const auto minDistTmp = getDistance(pt, doubleArea->corners[DartArea::Outer1]);
      const auto minDistTmp2 = getDistance(pt, doubleArea->corners[DartArea::Outer2]);
      if (minDistTmp > minDistance && minDistTmp2 > minDistance)
      {
        auto distTmp = getDistance(pt, outerPoint);
        if (getDistance(pt, doubleArea->significantPoints[0]) < maxDistanceSig0 && distTmp > maxDist1)
        {
          maxDist1 = distTmp;
          ptTmp1 = pt;
        }
        distTmp = getDistance(pt, outerPoint);
        if (getDistance(pt, doubleArea->significantPoints[1]) < maxDistanceSig1 && distTmp > maxDist2)
        {
          maxDist2 = distTmp;
          ptTmp2 = pt;
        }
      }
    }
    // Reversed order because Inner1 and Outer1 should be on the same side, but the algorithm alway gets the diagonally furthest away
    doubleArea->corners[DartArea::Inner2] = ptTmp1;
    doubleArea->corners[DartArea::Inner1] = ptTmp2;
  }
}


// Innerbullseye needed!
void DartBoard::setSortedMeanCorners(DartAreaArray areas)
{
  for (int i = 0; i < 10; i++)
  {
    //Only reds
    DartArea* dartArea = areas[i * 2];

    // Outer first, then inner
    for (auto j = 0; j < 2; j++)
    {
      const int idx1 = j * 2 + 0; // Outer / Inner 1
      const int idx2 = j * 2 + 1; // Outer / Inner 2

      const std::array<cv::Point, 4> neighbourPts = {
dartArea->neighbours[0]->corners[idx1],
dartArea->neighbours[0]->corners[idx2],
dartArea->neighbours[1]->corners[idx1],
dartArea->neighbours[1]->corners[idx2] };

      std::array<cv::Point, 2> pts = {
        dartArea->corners[idx1],
        dartArea->corners[idx2] };

      // Finds nearst 2 pts of the neighbourpoints to pts (one for each)
      unsigned distances[2] = {INT16_MAX, INT16_MAX};

      std::array<cv::Point, 2> outPts;
      for (auto i = 0; i < 4; i++)
      {
        const cv::Point cPoint = neighbourPts[i];

        for (auto k = 0; k < 2; k++)
        {
          const int distTmp = getDistance(pts[k], cPoint);
          if (distTmp < distances[k])
          {
            distances[k] = distTmp;
            outPts[k] = cPoint;
          }
        }
      }
   
      //auto out = findPoints(neighbourPts, pts, true);

      // Calculates mean of the 2 points that have 2 points next to each other
      std::array<cv::Point, 2> meanPoints = { (pts[0] + outPts[0]) / 2, (pts[1] + outPts[1]) / 2 };

      // Sorts the meanpoints in (anti)clockwise direction
      auto sortedMeanPoints = sortClockwise(singleBullCenter, meanPoints);
      dartArea->meanCorners[idx1] = sortedMeanPoints[0];
      dartArea->meanCorners[idx2] = sortedMeanPoints[1];
    }
  }
}

void DartBoard::setSingles()
{
  // Looks for hit in white
  for (auto i = 0; i < 10; i++)
  {
    auto areaDouble = doubles[i * 2];
    auto areaTriple = triples[i * 2];

    auto clockwiseNeighbourDouble = doubles[(i == 9 ? 0 : i * 2 + 2)];
    auto clockwiseNeighbourTriple = triples[(i == 9 ? 0 : i * 2 + 2)];

    std::vector<cv::Point> pts = {
      areaDouble->meanCorners[DartArea::Outer1],
      areaDouble->meanCorners[DartArea::Outer2],
      areaTriple->meanCorners[DartArea::Outer2],
      areaTriple->meanCorners[DartArea::Inner2],
      singleBullCenter,
      areaTriple->meanCorners[DartArea::Inner1],
      areaTriple->meanCorners[DartArea::Outer1], };

    DartArea* single = new DartArea(pts, false); 
    single->name = DartAreaName(i * 2 + 1, DartAreaName::Single);
    singles[i * 2] = single;

    // Sets outlining points of DartArea
    pts = {
    areaDouble->meanCorners[DartArea::Outer1],
    clockwiseNeighbourDouble->meanCorners[DartArea::Outer2],
    clockwiseNeighbourTriple->meanCorners[DartArea::Outer2],
    clockwiseNeighbourTriple->meanCorners[DartArea::Inner2],
    singleBullCenter,
    areaTriple->meanCorners[DartArea::Inner1],
    areaTriple->meanCorners[DartArea::Outer1] };

    single = new DartArea(pts, false);
    single->name = DartAreaName(i * 2 + 2, DartAreaName::Single);
    singles[i * 2 + 1] = single;
  }
}

void DartBoard::setNames()
{
  for (int i = 0; i < 20; i++)
  {
    triples[i]->name = DartAreaName(i + 1, DartAreaName::Triple);

    doubles[i]->name = DartAreaName(i + 1, DartAreaName::Double);
  }

  bullseye.name = DartAreaName(1, DartAreaName::Bullseye);
  singleBull.name = DartAreaName(1, DartAreaName::SingleBull);
}

void DartBoard::setTitlePoint()
{
  titlePoint = cv::Point(singleBullCenter.x, singleBullCenter.y - 1.25 * doubles[0]->meanPoint.y);
}

void DartBoard::drawBoard(cv::Mat& img, cv::Size sizeReference)
{
  if (img.empty())
  {
    img = cv::Mat::zeros(sizeReference, CV_8UC3);
  }

  // Temporary contours safe
  std::vector<std::vector<cv::Point>> contoursBuff;
  for (auto dartArea : doubles)
  {
    contoursBuff.push_back(dartArea->contour);
  }

  for (auto dartArea : triples)
  {
    contoursBuff.push_back(dartArea->contour);
  }

  if (!singleBull.contour.empty() && !bullseye.contour.empty())
  {
    contoursBuff.push_back(singleBull.contour);
    contoursBuff.push_back(bullseye.contour);
  }

  for (size_t i = 0; i < contoursBuff.size(); i++)
  {
    drawContours(img, contoursBuff, static_cast<int>(i), i % 2 ? _redColor : _greenColor, 2);
  }
}

DartArea* DartBoard::detectHit(const cv::Point point)
{
  // Looks for hit in triples
  for (auto area : triples)
  {
    if (pointPolygonTest(area->contour, point, false) >= 0)
    {
      return area;
    }
  }

  // Looks for hit in doubles
  for (auto area : doubles)
  {
    if (pointPolygonTest(area->contour, point, false) >= 0)
    {
      return area;
    }
  }

  // Looks for hit in bullseye
  if (pointPolygonTest(bullseye.contour, point, false) >= 0)
  {
    return &bullseye;
  }
  if (pointPolygonTest(singleBull.contour, point, false) >= 0)
  {
    return &singleBull;
  }

  // Looks for hit in singles
  // Has to be checked last because the singles are also in the area of the doubles / triples
  for (auto area : singles)
  {
    if (pointPolygonTest(area->contour, point, false) >= 0)
    {
      return area;
    }
  }
  
  return nullptr;
}

void DartBoard::printText(cv::Mat& img, DartAreaArray areas, const std::string prefix, int fontFace, int fontScale, const cv::Scalar color, int thickness)
{
  for (int i = 0; i < 20; i++)
  {
    std::string text = prefix;
    text += std::to_string(DartAreaMapping.at(i + 1));
    cv::putText(img, text, areas[i]->meanPoint, fontFace, fontScale, color, thickness);
  }
}

bool DartBoard::isReady() const
{
  return ready;
}