#pragma once
#define _WCHAR_H_CPLUSPLUS_98_CONFORMANCE_

#ifdef __APPLE__
    #include <sys/uio.h>
	#include <unistd.h>
#else
    #include <io.h>
	#define access _access
#endif

#include "textfile.h"
#include <rapidxml.hpp>
#include <string>
#include <unordered_map>

using namespace rapidxml;

class PointCloudUtils;

class ConfigManager
{
private:
    std::string getFolder(const std::string& str) const
	{
		unsigned pos1 = str.rfind('/');
		unsigned pos2 = str.rfind('\\');
		unsigned pos = pos1 < pos2 ? pos1 : pos2;
		if(pos < str.size())
			return str.substr(0, pos);
		return "";
	}

    std::string rootFolder;

    std::string currentPath;

    std::unordered_map<std::string, std::pair<xml_document<>*, char*> > path_doc;

	PointCloudUtils* pcUtils;

	xml_node<>* findNode(const std::string& filePath, const std::string& nodeTag,
        const std::string& nodeName);

	xml_node<>* findNode(xml_node<>* root, const std::string& nodeTag,
        const std::string& nodeName);

    std::string getPath(xml_node<>* node);
    std::pair<std::string, std::string> getPathAndName(xml_node<>* node);

	void clear()
	{
		for(std::unordered_map<std::string, std::pair<xml_document<>*, char*> >::iterator it=path_doc.begin(); it!=path_doc.end(); it++)
		{
			delete (it->second).first;
			free(it->second.second);
		}
		path_doc.clear();
	}

public:
    std::string getFullPath(const std::string& fn) const
	{
		if(access(fn.c_str(), 0) == 0)
			return fn;
        std::string fullPath = getFolder(rootFolder + "/" + currentPath) + "/" + fn;
		if(access(fullPath.c_str(), 0) == 0)
			return fullPath;
		fullPath = rootFolder + "/" + fn;
		if(access(fullPath.c_str(), 0) == 0)
			return fullPath;
		return "";
	}
	ConfigManager(PointCloudUtils* _pcUtils);
	void load(const std::string &configFilePath);
    std::string getRootPath() const { return rootFolder; }
};

