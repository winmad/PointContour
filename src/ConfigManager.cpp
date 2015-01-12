#include "ConfigManager.h"
#include "PointCloudUtils.h"
using namespace std;

ConfigManager::ConfigManager(PointCloudUtils* _pcUtils)
{
	this->pcUtils = _pcUtils;
}

void ConfigManager::load(const string &configFilePath)
{
	clear();
	
	pair<string, string> path_name;
    string pathStr;

	rootFolder = getFolder(configFilePath);

	xml_node<> *nodeConfig = findNode(configFilePath, "config", "");

    if (nodeConfig->first_node("matlab_files"))
    {
        pathStr = getPath(nodeConfig->first_node("matlab_files"));
        printf("matlab_files_path: %s\n" , pathStr.c_str());
        pcUtils->matlabFilesPath = pathStr;
    }

    if (nodeConfig->first_node("ampl_exe"))
    {
        pathStr = getPath(nodeConfig->first_node("ampl_exe"));
        printf("ampl_files_path: %s\n" , pathStr.c_str());
        pcUtils->opt.amplExePath = pathStr;
    }

    if (nodeConfig->first_node("ampl_include"))
    {
        pathStr = getPath(nodeConfig->first_node("ampl_include"));
        printf("ampl_include_path: %s\n" , pathStr.c_str());
        pcUtils->opt.amplIncludePath = pathStr;
    }

    if (nodeConfig->first_node("ampl_result"))
    {
        pathStr = getPath(nodeConfig->first_node("ampl_result"));
        printf("ampl_result_path: %s\n" , pathStr.c_str());
        pcUtils->opt.amplResultPath = pathStr;
    }

    if (nodeConfig->first_node("data_curve"))
    {
        pathStr = getPath(nodeConfig->first_node("data_curve"));
        printf("data_curve_path: %s\n" , pathStr.c_str());
        pcUtils->dataCurvePath = pathStr;
    }

    if (nodeConfig->first_node("data_votes"))
    {
        pathStr = getPath(nodeConfig->first_node("data_votes"));
        printf("data_votes_path: %s\n" , pathStr.c_str());
        pcUtils->dataVotesPath = pathStr;
    }

    /*
	if (nodeConfig->first_node("useGPU"))
	{
		int flag = atoi(nodeConfig->first_node("useGPU")->value());
		if (flag == 1)
			renderer->scene.setGPU(true);
	}

	xml_node<> *nodeSceneConfig = nodeConfig->first_node("Scene");

	path_name = getPathAndName(nodeSceneConfig);
    // printf("scene path = %s , %s\n" , path_name.first.c_str() , path_name.second.c_str());
	xml_node<> *nodeScene = findNode(path_name.first, "Scene", path_name.second);
	if(!nodeScene)
		nodeScene = findNode(configFilePath, "Scene", path_name.second);

	currentPath = path_name.first;

	for(xml_node<>* nodeGroupObj = nodeScene->first_node("GroupObject"); nodeGroupObj; nodeGroupObj = nodeGroupObj->next_sibling("GroupObject"))
	{
		string path = nodeGroupObj->first_node("filePath")->value();
		
		if(access(path.c_str(), 0) == -1)
		{
			path = rootFolder + '/' + path;
		}

		SimpleShape ss;
		if(nodeGroupObj->first_node("transform"))
			ss.setTransform(readMatrix(nodeGroupObj->first_node("transform")->value()));
		vector<SimpleShape*> shapes;
		ss.loadShape(path, true, &shapes);

		unordered_map<string, SimpleShape*> name_shape;

		for(unsigned i=0; i<shapes.size(); i++)
			name_shape[shapes[i]->name] = shapes[i];
		
		for(xml_node<>* nodeObj = nodeGroupObj->first_node("Object"); nodeObj; nodeObj = nodeObj->next_sibling("Object"))
		{
			path_name = getPathAndName(nodeObj->first_node("Material"));
			xml_node<>* nodeMat = findNode(path_name.first, "Material", path_name.second);
			if(!nodeMat && nodeObj->first_node("Material")->first_node("name"))
				nodeMat = findNode(nodeScene, "Material", path_name.second);
			if(!nodeMat)
				nodeMat = nodeObj->first_node("Material");
			SimpleShape *shape = name_shape[nodeObj->first_attribute("Name")->value()];

			renderer->scene.objects.push_back(generateSceneObject(nodeObj, nodeMat, shape));
		}
		for(unsigned i=0; i<shapes.size(); i++)
			delete shapes[i];
	}

	for(xml_node<>* nodeObj = nodeScene->first_node("Object"); nodeObj; nodeObj = nodeObj->next_sibling("Object"))
	{
		path_name = getPathAndName(nodeObj->first_node("Material"));
		xml_node<>* nodeMat = findNode(path_name.first, "Material", path_name.second);
		if(!nodeMat && nodeObj->first_node("Material")->first_node("name"))
			nodeMat = findNode(nodeScene, "Material", path_name.second);
		if(!nodeMat)
			nodeMat = nodeObj->first_node("Material");

		string path = nodeObj->first_node("filePath")->value();
		if(access(path.c_str(), 0) == -1)
		{
			path = rootFolder + '/' + path;
		}

		SceneObject *obj = generateSceneObject(nodeObj, nodeMat);

		obj->loadShape(path , true); // NEED TO BE TRUE WHEN NOT TESTING
		// FALSE WHEN test_cornell_box

		if(obj->isVolumetric()){
			if(nodeMat->first_node("density_file"))
				((HeterogeneousVolume*)obj)->loadDensityMap(nodeMat->first_node("density_file")->value());

			if(nodeMat->first_node("volume_data_s") && nodeMat->first_node("volume_data_a")){
				((HeterogeneousVolume*)obj)->loadSubSurfaceVolumeData(nodeMat->first_node("volume_data_s")->value(), nodeMat->first_node("volume_data_a")->value());
			}
			std::cout << "BBox = " << (vec3f(obj->transform * vec4f(obj->minCoord, 1))) << " " << (vec3f(obj->transform * vec4f(obj->maxCoord, 1))) << std::endl;
		}


		renderer->scene.objects.push_back(obj);
	}

	xml_node<> *nodeCam = findNode(nodeScene, "Camera", nodeSceneConfig->first_node("camera")->value());
	renderer->camera.width = atoi(nodeCam->first_node("width")->value());
	renderer->camera.height = atoi(nodeCam->first_node("height")->value());
	renderer->camera.sightDist = atoi(nodeCam->first_node("sightDist")->value());
	renderer->camera.focus = readVec(nodeCam->first_node("focus")->value());
	renderer->camera.position = readVec(nodeCam->first_node("position")->value());
	renderer->camera.up = readVec(nodeCam->first_node("up")->value());

	renderer->scene.buildKDTree();

	if(renderer->mcRenderer)
	{
		delete renderer->mcRenderer;
		renderer->mcRenderer = NULL;
	}

	if(nodeConfig->first_node("renderer"))
	{
		string typeName = nodeConfig->first_node("renderer")->value();
		if(typeName == "PathTracer" || typeName == "PT")
		{
			renderer->mcRenderer = new PathTracer(renderer);
		}
		if(typeName == "PTtest")
		{
			renderer->mcRenderer = new PathTracerTest(renderer);
		}
		if(typeName == "PhotonMap" || typeName == "PM")
		{
			renderer->mcRenderer = new PhotonMap(renderer);
			//static_cast<PhotonMap*>(renderer->mcRenderer)->setPhotonsWant(atoi(nodeConfig->first_node("photons")->value()));
			static_cast<PhotonMap*>(renderer->mcRenderer)->setRadius(atof(nodeConfig->first_node("radius")->value()));
		}
		if(typeName == "BidirectionalPathTracer" || typeName == "BPT")
		{
			renderer->mcRenderer = new BidirectionalPathTracer(renderer);
		}
		//if (typeName == "newBPT")
		//{
		//	renderer->mcRenderer = new NewBidirectionalPathTracer(renderer);
		//}
		if(typeName == "VCMTracer" || typeName == "VCM")
		{
			renderer->mcRenderer = new VCMTracer(renderer);
			((VCMTracer*)renderer->mcRenderer)->setRadius(atof(nodeConfig->first_node("radius")->value()));
		}
		if(typeName == "IptTracer" || typeName == "IPT")
		{
			renderer->mcRenderer = new IptTracer(renderer);
			((IptTracer*)renderer->mcRenderer)->setRadius(atof(nodeConfig->first_node("radius")->value()));
			if (nodeConfig->first_node("gatherRadius"))
				((IptTracer*)renderer->mcRenderer)->gatherRadius = (atof(nodeConfig->first_node("gatherRadius")->value()));
		}
		if (nodeConfig->first_node("maxDepth"))
			renderer->mcRenderer->setMaxDepth(atoi(nodeConfig->first_node("maxDepth")->value()));
	}

	xml_node<>* nodeRenderer = nodeConfig->first_node("Renderer");

	if(nodeRenderer)
	{
		string typeName = nodeRenderer->first_node("type")->value();
		if(typeName == "PathTracer" || typeName == "PT")
		{
			renderer->mcRenderer = new PathTracer(renderer);
		}
		if(typeName == "PTtest")
		{
			renderer->mcRenderer = new PathTracerTest(renderer);
		}
		if(typeName == "PhotonMap" || typeName == "PM")
		{
			renderer->mcRenderer = new PhotonMap(renderer);
			//if(nodeRenderer->first_node("photons"))
			//	static_cast<PhotonMap*>(renderer->mcRenderer)->setPhotonsWant(atoi(nodeRenderer->first_node("photons")->value()));
			if(nodeRenderer->first_node("radius"))
				static_cast<PhotonMap*>(renderer->mcRenderer)->setRadius(atof(nodeRenderer->first_node("radius")->value()));
		}
		if(typeName == "BidirectionalPathTracer" || typeName == "BPT")
		{
			renderer->mcRenderer = new BidirectionalPathTracer(renderer);
		}
		//if (typeName == "newBPT")
		//{
		//	renderer->mcRenderer = new NewBidirectionalPathTracer(renderer);
		//}
		if(typeName == "VCMTracer" || typeName == "VCM")
		{
			renderer->mcRenderer = new VCMTracer(renderer);
			if(nodeRenderer->first_node("radius"))
				((VCMTracer*)renderer->mcRenderer)->setRadius(atof(nodeRenderer->first_node("radius")->value()));
		}
		if(typeName == "IptTracer" || typeName == "IPT")
		{
			renderer->mcRenderer = new IptTracer(renderer);
			if(nodeRenderer->first_node("radius"))
				((IptTracer*)renderer->mcRenderer)->setRadius(atof(nodeRenderer->first_node("radius")->value()));
			if (nodeRenderer->first_node("gatherRadius"))
				((IptTracer*)renderer->mcRenderer)->gatherRadius = (atof(nodeRenderer->first_node("gatherRadius")->value()));
			if (nodeRenderer->first_node("pathRatio"))
				((IptTracer*)renderer->mcRenderer)->pathRatio = (atof(nodeRenderer->first_node("pathRatio")->value())); 
			if (nodeRenderer->first_node("mergeRatio"))
				((IptTracer*)renderer->mcRenderer)->mergeRatio = (atof(nodeRenderer->first_node("mergeRatio")->value())); 
			if (nodeRenderer->first_node("usePPM"))
				((IptTracer*)renderer->mcRenderer)->usePPM = (atoi(nodeRenderer->first_node("usePPM")->value()) == 1 ? true : false);
			if (nodeRenderer->first_node("pathNum"))
				((IptTracer*)renderer->mcRenderer)->totPathNum = (atoi(nodeRenderer->first_node("pathNum")->value()));
			if (nodeRenderer->first_node("isDebug"))
				((IptTracer*)renderer->mcRenderer)->isDebug = (atoi(nodeRenderer->first_node("isDebug")->value()) == 1 ? true : false);
			if (nodeRenderer->first_node("useUniformSur"))
				((IptTracer*)renderer->mcRenderer)->useUniformSur = (atoi(nodeRenderer->first_node("useUniformSur")->value()) == 1 ? true : false);
			if (nodeRenderer->first_node("useDirIllu"))
				((IptTracer*)renderer->mcRenderer)->useDirIllu = (atoi(nodeRenderer->first_node("useDirIllu")->value()) == 1 ? true : false);
		}
		if (nodeRenderer->first_node("maxDepth"))
			renderer->mcRenderer->setMaxDepth(atoi(nodeRenderer->first_node("maxDepth")->value()));
        if (nodeRenderer->first_node("spp"))
            renderer->mcRenderer->setSpp(atoi(nodeRenderer->first_node("spp")->value()));
    }

    xml_node<> *nodeTime = nodeConfig->first_node("time");
    if (nodeTime)
    {
        double runtime = 0.0;
        if (nodeTime->first_node("hours"))
        {
            runtime += 3600.0 * atoi(nodeTime->first_node("hours")->value());
        }
        if (nodeTime->first_node("minutes"))
        {
            runtime += 60.0 * atoi(nodeTime->first_node("minutes")->value());
        }
        // printf("set runtime = %.1lfs\n" , runtime);
        renderer->mcRenderer->setRuntime(runtime);
    }

    if(nodeConfig->first_node("savePath"))
	{
		renderer->mcRenderer->setSavePath(nodeConfig->first_node("savePath")->value());
	}
    */
}

xml_node<>* ConfigManager::findNode(const string& filePath, const string& nodeTag, const string& nodeName)
{
	if(filePath == "")
		return NULL;
	string fullFilePath = filePath;
	if(access(filePath.c_str(), 0) == -1)
		fullFilePath = rootFolder + string("/") + filePath;
	xml_document<> *doc;
	if(path_doc.find(fullFilePath) != path_doc.end())
	{
		doc = path_doc[fullFilePath].first;
	}
	else
	{
		doc = new xml_document<>;
		char* text = textFileRead(fullFilePath.c_str());
		path_doc[fullFilePath] = make_pair(doc, text);
		doc->parse<0>(text);
	}
	for(xml_node<> *node=doc->first_node(nodeTag.c_str()); node; node = node->next_sibling(nodeTag.c_str()))
	{
		if(node->first_attribute("Name") == NULL || nodeName == "")
			return node;
		if(node->first_attribute("Name") && node->first_attribute("Name")->value() == nodeName)
			return node;
	}
	return NULL;
}

xml_node<>* ConfigManager::findNode(xml_node<>* root, const string& nodeTag, const string& nodeName)
{
	for(xml_node<> *node=root->first_node(nodeTag.c_str()); node; node = node->next_sibling(nodeTag.c_str()))
	{
		if(node->first_attribute("Name") == NULL || nodeName == "")
			return node;
		if(node->first_attribute("Name") && node->first_attribute("Name")->value() == nodeName)
			return node;
	}
	return NULL;
}

string ConfigManager::getPath(xml_node<>* node)
{
    string str;
    str = node->first_node("path") ? node->first_node("path")->value() : currentPath;
    return str;
}

pair<string, string> ConfigManager::getPathAndName(xml_node<>* node)
{
	pair<string, string> path_name;
	path_name.first = node->first_node("path") ? node->first_node("path")->value() : currentPath;
	path_name.second = node->first_node("name") ? node->first_node("name")->value() : "";
	return path_name;
}