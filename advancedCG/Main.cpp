

#include "GEMLoader.h"
#include "Renderer.h"
#include "SceneLoader.h"
#define NOMINMAX
#include "GamesEngineeringBase.h"
#include <unordered_map>

void runTests()
{
	Plane p;
	Vec3 v(0, 1, 0);
	p.init(v, 0);

	Ray r(Vec3(0, 5, 0), Vec3(0, -1, 0));

	float t = 0.f;
	bool a = p.rayIntersect(r, t);
	
	if (a) {
		std::cout << "Hit at " << r.at(t).x << " " << r.at(t).y << " " << r.at(t).z << std::endl;
	}
	else {
		std::cout << "No hit" << std::endl;
	}

}

int main(int argc, char *argv[])
{
	// Add call to tests if required
	runTests();
	
	// Initialize default parameters
	std::string sceneName = "cornell-box";
	std::string filename = "GI.hdr";
	unsigned int SPP = 8192;

	if (argc > 1)
	{
		std::unordered_map<std::string, std::string> args;
		for (int i = 1; i < argc; ++i)
		{
			std::string arg = argv[i];
			if (!arg.empty() && arg[0] == '-')
			{
				std::string argName = arg;
				if (i + 1 < argc)
				{
					std::string argValue = argv[++i];
					args[argName] = argValue;
				} else
				{
					std::cerr << "Error: Missing value for argument '" << arg << "'\n";
				}
			} else
			{
				std::cerr << "Warning: Ignoring unexpected argument '" << arg << "'\n";
			}
		}
		for (const auto& pair : args)
		{
			if (pair.first == "-scene")
			{
				sceneName = pair.second;
			}
			if (pair.first == "-outputFilename")
			{
				filename = pair.second;
			}
			if (pair.first == "-SPP")
			{
				SPP = stoi(pair.second);
			}
		}
	}
	Scene* scene = loadScene(sceneName);
	GamesEngineeringBase::Window canvas;
	canvas.create((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, "Tracer", false);
	RayTracer rt;
	rt.init(scene, &canvas);
	bool running = true;
	GamesEngineeringBase::Timer timer;
	while (running)
	{
		canvas.checkInput();
		canvas.clear();
		if (canvas.keyPressed(VK_ESCAPE))
		{
			break;
		}
		if (canvas.keyPressed('W'))
		{
			viewcamera.forward();
			rt.clear();
		}
		if (canvas.keyPressed('S'))
		{
			viewcamera.back();
			rt.clear();
		}
		if (canvas.keyPressed('A'))
		{
			viewcamera.left();
			rt.clear();
		}
		if (canvas.keyPressed('D'))
		{
			viewcamera.right();
			rt.clear();
		}
		if (canvas.keyPressed('E'))
		{
			viewcamera.flyUp();
			rt.clear();
		}
		if (canvas.keyPressed('Q'))
		{
			viewcamera.flyDown();
			rt.clear();
		}
		// Time how long a render call takes
		timer.reset();
		rt.render();
		float t = timer.dt();
		// Write
		std::cout << t << std::endl;
		if (canvas.keyPressed('P'))
		{
			rt.saveHDR(filename);
		}
		if (canvas.keyPressed('L'))
		{
			size_t pos = filename.find_last_of('.');
			std::string ldrFilename = filename.substr(0, pos) + ".png";
			rt.savePNG(ldrFilename);
		}
		if (SPP == rt.getSPP())
		{
			rt.saveHDR(filename);
			break;
		}
		canvas.present();
	}
	return 0;
}