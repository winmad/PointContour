/*************************************************************************
*
* ADOBE PROPRIETARY INFORMATION
*
* Use is governed by the license in the attached LICENSE.TXT file
*
*
*  Copyright 2010 Adobe Systems Incorporated
*  All Rights Reserved.
*
* NOTICE:  All information contained herein is, and remains
* the property of Adobe Systems Incorporated and its suppliers,
* if any.  The intellectual and technical concepts contained
* herein are proprietary to Adobe Systems Incorporated and its
* suppliers and may be covered by U.S. and Foreign Patents,
* patents in process, and are protected by trade secret or copyright law.
* Dissemination of this information or reproduction of this material
* is strictly forbidden unless prior written permission is obtained
* from Adobe Systems Incorporated.
**************************************************************************/
//#pragma once
#ifndef __FILE_IO_HELPER_H
#define __FILE_IO_HELPER_H

#include <fstream>
#include <vector>
#include <iostream>
#include "nvVector.h"

using namespace std;


#define MAX_LINE_LENGTH 2048 // used in the maya file reader


class FileIOHelper
{

public:
	//public:
	FileIOHelper();
	~FileIOHelper();

	bool readPointCloudDataNpts(
		const char* fileName,
		std::vector<vec3f>& points,
		std::vector<vec3f>& normals);

	bool readPointCloudDataPly(
		const char* fileName,
		std::vector<vec3f>& points);
};

#endif
