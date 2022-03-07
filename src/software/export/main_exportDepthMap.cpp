// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2015 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/stl/regex.hpp>
#include <aliceVision/config.hpp>
#include <aliceVision/utils/regexFilter.hpp>
#include <aliceVision/mvsUtils/fileIO.hpp>
#include <aliceVision/mvsData/imageIO.hpp>
#include <aliceVision/mvsData/Matrix3x3.hpp>
#include <aliceVision/mvsData/Point2d.hpp>
#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsData/jetColorMap.hpp>
#include <OpenImageIO/imagebufalgo.h>

#include <boost/program_options.hpp>
#include <boost/system/error_code.hpp>
#include <boost/filesystem.hpp>

#include <algorithm>
#include <string>
#include <regex>
#include <iostream>
#include <fstream>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 2
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

void writeDepthMapToPly(const oiio::ImageBuf& inBuf, const std::string& outputPlyFilename)
{
  const oiio::ImageSpec& inSpec = inBuf.spec();

  std::cout << "[DepthMapEntity] Image Size: " << inSpec.width << "x" << inSpec.height << std::endl;

  Point3d CArr;
  const oiio::ParamValue * cParam = inSpec.find_attribute("AliceVision:CArr"); // , oiio::TypeDesc(oiio::TypeDesc::DOUBLE, oiio::TypeDesc::VEC3));
  if(cParam)
  {
      std::cout << "[DepthMapEntity] CArr: " << cParam->nvalues() << std::endl;
      std::copy_n((const double*)cParam->data(), 3, CArr.m);
  }
  else
  {
      std::cout << "[DepthMapEntity] Error: missing metadata CArr." << std::endl;
      return;
  }

  Matrix3x3 iCamArr;
  const oiio::ParamValue * icParam = inSpec.find_attribute("AliceVision:iCamArr", oiio::TypeDesc(oiio::TypeDesc::DOUBLE, oiio::TypeDesc::MATRIX33));
  if(icParam)
  {
      std::cout << "[DepthMapEntity] iCamArr: " << icParam->nvalues() << std::endl;
      std::copy_n((const double*)icParam->data(), 9, iCamArr.m);
  }
  else
  {
      std::cout << "[DepthMapEntity] Error: missing metadata iCamArr." << std::endl;
      return;
  }

  // Get number of non empty points
  int numPoints = 0;
  for(int y = 0; y < inSpec.height; ++y)
  {
      for(int x = 0; x < inSpec.width; ++x)
      {
          float depthValue = 0.0f;
          inBuf.getpixel(x, y, &depthValue, 1);
          if(depthValue <= 0.f)
              continue;

          numPoints++;
      }
  }

  oiio::ImageBufAlgo::PixelStats stats;
  oiio::ImageBufAlgo::computePixelStats(stats, inBuf);

  // Write out PLY header
  std::ofstream plyFile(outputPlyFilename);

  plyFile << "ply" << std::endl;

  plyFile << "format ascii 1.0" << std::endl;
  plyFile << "element vertex " << numPoints << std::endl;
  plyFile << "property float x" << std::endl;
  plyFile << "property float y" << std::endl;
  plyFile << "property float z" << std::endl;
  plyFile << "property uchar red" << std::endl;
  plyFile << "property uchar green" << std::endl;
  plyFile << "property uchar blue" << std::endl;
  plyFile << "end_header" << std::endl;

  // Write out vertices
  for(int y = 0; y < inSpec.height; ++y)
  {
      for(int x = 0; x < inSpec.width; ++x)
      {
          float depthValue = 0.0f;
          inBuf.getpixel(x, y, &depthValue, 1);
          if(depthValue <= 0.f)
              continue;

          Point3d p = CArr + (iCamArr * Point2d((double)x, (double)y)).normalize() * depthValue;
          //Vec3f position(p.x, p.y, p.z);

          //indexPerPixel[y * inSpec.width + x] = positions.size();
          //positions.push_back(position);

          const float range = stats.max[0] - stats.min[0];
          float normalizedDepthValue = range != 0.0f ? (depthValue - stats.min[0]) / range : 1.0f;
          rgb color = getRGBFromJetColorMap(normalizedDepthValue);

          plyFile << p.x << " " << p.y << " " << p.z << " " << (int)color.r << " " << (int)color.g << " " << (int)color.b << std::endl;
      }
  }

  plyFile.close();

  //auto numPoints = std::count_if(data.begin(), data.end(),[](auto const& val){ return val > 0.0f; }); // Some values of -1 think that's out of bounds?

  //printf("We have %i points out of %i possible\n", numPoints, width*height);
}

// Export depth map to PLY format for debugging
int aliceVision_main(int argc, char **argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::string depthMapsFolder;
  std::string outputPlyFilename;
  std::string viewId;

  po::options_description allParams("AliceVision exportDepthMap");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file.")
    ("depthMapsFolder", po::value<std::string>(&depthMapsFolder)->required(),
            "Input depth map folder.")
    ("output,o", po::value<std::string>(&outputPlyFilename)->required(),
      "Path to the output Alembic file.")
    ("viewId", po::value<std::string>(&viewId)->required(),
      "View Id to extract");

  po::options_description optionalParams("Optional parameters");
  // optionalParams.add_options()
  //   ("describerTypes,d", po::value<std::string>(&describerTypesName)->default_value(describerTypesName),
  //     feature::EImageDescriberType_informations().c_str())
  //   ("imageWhiteList", po::value<std::vector<std::string>>(&imageWhiteList)->multitoken()->default_value(imageWhiteList),
  //     "image white list containing uid(s), image filenames or regex(es) on the image file path (supported regex: '#' matches a single digit, '@' one or more digits, '?' one character and '*' zero or more)")
  //   ("views", po::value<bool>(&flagViews)->default_value(flagViews),
  //     "Export views.")
  //   ("intrinsics", po::value<bool>(&flagIntrinsics)->default_value(flagIntrinsics),
  //     "Export intrinsics.")
  //   ("extrinsics", po::value<bool>(&flagExtrinsics)->default_value(flagExtrinsics),
  //     "Export extrinsics.")
  //   ("structure", po::value<bool>(&flagStructure)->default_value(flagStructure),
  //     "Export structure.")
  //   ("observations", po::value<bool>(&flagObservations)->default_value(flagObservations),
  //     "Export observations.");

  po::options_description logParams("Log parameters");
  logParams.add_options()
    ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
      "verbosity level (fatal,  error, warning, info, debug, trace).");

  allParams.add(requiredParams).add(optionalParams).add(logParams);

  po::variables_map vm;
  try
  {
    po::store(po::parse_command_line(argc, argv, allParams), vm);

    if(vm.count("help") || (argc == 1))
    {
      ALICEVISION_COUT(allParams);
      return EXIT_SUCCESS;
    }
    po::notify(vm);
  }
  catch(boost::program_options::required_option& e)
  {
    ALICEVISION_CERR("ERROR: " << e.what());
    ALICEVISION_COUT("Usage:\n\n" << allParams);
    return EXIT_FAILURE;
  }
  catch(boost::program_options::error& e)
  {
    ALICEVISION_CERR("ERROR: " << e.what());
    ALICEVISION_COUT("Usage:\n\n" << allParams);
    return EXIT_FAILURE;
  }

  ALICEVISION_COUT("Program called with the following parameters:");
  ALICEVISION_COUT(vm);

  // set verbose level
  system::Logger::get()->setLogLevel(verboseLevel);

  if(sfmDataFilename.empty() || outputPlyFilename.empty())
  {
    ALICEVISION_LOG_ERROR("Invalid input or output filename");
    return EXIT_FAILURE;
  }

  // load input SfMData scene
  sfmData::SfMData sfmData;
  if(!sfmDataIO::Load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read");
    return EXIT_FAILURE;
  }

  // Setup some dummy multiview params
  mvsUtils::MultiViewParams mp(sfmData, "", depthMapsFolder, "", /*readFromDepthMaps=*/true);

  // Read in the depth map
  std::vector<float> depthMap;
  int width, height;
  int camIdx = mp.getIndexFromViewId(std::atoi(viewId.c_str()));
  printf("CamIdx = %i\n", camIdx);

  std::string depthMapFilename = getFileNameFromIndex(mp, camIdx, mvsUtils::EFileType::depthMap, 1);

  // Read in using method from QtOIIO plugin
  oiio::ImageSpec configSpec;
  // libRAW configuration
  configSpec.attribute("raw:auto_bright", 0);       // don't want exposure correction
  configSpec.attribute("raw:use_camera_wb", 1);     // want white balance correction
  configSpec.attribute("raw:ColorSpace", "sRGB");   // want colorspace sRGB
  configSpec.attribute("raw:use_camera_matrix", 3); // want to use embeded color profile

  oiio::ImageBuf inBuf(depthMapFilename, 0, 0, NULL, &configSpec);
  
  writeDepthMapToPly(inBuf, outputPlyFilename);

  return EXIT_SUCCESS;
}
