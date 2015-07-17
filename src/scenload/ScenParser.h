/*
 * ScenParser.h
 *
 *  Created on: May 21, 2012
 *      Author: doancea
 */

#ifndef SCENPARSER_H_
#define SCENPARSER_H_

#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

class ScenParser
{
 public:
  ScenParser(const char* filePath);
  ~ScenParser();
 private:
  ScenParser();
};
#endif /* SCENPARSER_H_ */