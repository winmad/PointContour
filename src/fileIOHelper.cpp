#include "fileIOHelper.h"
#include <string.h>
#include <string>

FileIOHelper::FileIOHelper()
{
	//m_SvgLoader = new SimpleSVGLoader(); //NOSVG
}

FileIOHelper::~FileIOHelper()
{
	//delete m_SvgLoader; //NOSVG
}

bool FileIOHelper::readPointCloudDataNpts(const char* fileName, 
									  std::vector<vec3f>& points,
									  std::vector<vec3f>& normals)
{
	if (!fileName)
		return false;

	std::ifstream reader(fileName);

	if (!reader.good())
		return false;

	points.clear();
	normals.clear();

	vec3f p , n;

	while (reader >> p.x >> p.y >> p.z >> n.x >> n.y >> n.z)
	{
		points.push_back(p);
		normals.push_back(n);
	}

	reader.close();
	return true;
}

bool FileIOHelper::readPointCloudDataPly(const char* fileName, 
										std::vector<vec3f>& points)
{
	if (!fileName)
		return false;

	std::ifstream reader(fileName);

	if (!reader.good())
		return false;

	points.clear();

	vec3f p;

	std::string str;
	while (str != "end_header")
	{
		getline(reader , str);
	}

	while (reader >> p.x >> p.y >> p.z)
	{
		points.push_back(p);
	}

	reader.close();
	return true;
}
