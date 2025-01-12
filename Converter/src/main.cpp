#include <iostream>
#include <execution>

#include "unsuck/unsuck.hpp"
#include "chunker_countsort_laszip.h"
#include "indexer.h"
#include "sampler_poisson.h"
#include "sampler_poisson_average.h"
#include "sampler_random.h"
#include "Attributes.h"
#include "PotreeConverter.h"
#include "logger.h"
#include "Monitor.h"

#include "arguments/Arguments.hpp"

using namespace std;

Options parseArguments(int argc, char** argv) {
	Arguments args(argc, argv);

	args.addArgument("source,i,", "Input file(s)");
	args.addArgument("help,h", "Display help information");
	args.addArgument("outdir,o", "Output directory");
	args.addArgument("encoding", "Encoding type \"BROTLI\", \"UNCOMPRESSED\" (default)");
	args.addArgument("method,m", "Point sampling method \"poisson\", \"poisson_average\", \"random\"");
	args.addArgument("chunkMethod", "Chunking method");
	args.addArgument("keep-chunks", "Skip deleting temporary chunks during conversion");
	args.addArgument("no-chunking", "Disable chunking phase");
	args.addArgument("no-indexing", "Disable indexing phase");
	args.addArgument("attributes", "Attributes in output file");
	args.addArgument("projection", "Add the projection of the pointcloud to the metadata");
	args.addArgument("generate-page,p", "Generate a ready to use web page with the given name");
	args.addArgument("title", "Page title used when generating a web page");

	if (args.has("help")) {
		cout << "PotreeConverter <source> -o <outdir>" << endl;
		cout << endl << args.usage() << endl;
		exit(0);
	}

	if (!args.has("source")) {
		cout << "PotreeConverter <source> -o <outdir>" << endl;
		cout << endl << "For a list of options, use --help or -h" << endl;

		exit(1);
	}

	vector<string> source = args.get("source").as<vector<string>>();

	if (source.size() == 0) {
		cout << "PotreeConverter <source> -o <outdir>" << endl;
		cout << endl << "For a list of options, use --help or -h" << endl;

		exit(1);
	}

	string encoding = args.get("encoding").as<string>("DEFAULT");
	string method = args.get("method").as<string>("poisson");
	string chunkMethod = args.get("chunkMethod").as<string>("LASZIP");

	string outdir = "";
	if (args.has("outdir")) {
		outdir = args.get("outdir").as<string>();
	} else {

		string sourcepath = source[0];
		fs::path path(sourcepath);

		if (!fs::exists(path)) {

			logger::ERROR("file does not exist: " + source[0]);

			exit(123);
		}

		path = fs::canonical(path);

		const string suggestedBaseName = path.filename().string() + "_converted";
		outdir = sourcepath + "/../" + suggestedBaseName;

		int i = 1;
		while(fs::exists(outdir)) {
			outdir = sourcepath + "/../" + suggestedBaseName + "_" + std::to_string(i);

			if (i > 100) {

				logger::ERROR("unsuccessfully tried to find empty output directory. stopped at 100 iterations: " + outdir);

				exit(123);
			}

			i++;
		}

	}

	outdir = fs::weakly_canonical(fs::path(outdir)).string();

	vector<string> attributes = args.get("attributes").as<vector<string>>();

	const bool generatePage = args.has("generate-page");
	string pageName = "";
	if (generatePage) {
		pageName = args.get("generate-page").as<string>();
	}
	const string pageTitle = args.get("title").as<string>();
	const string projection = args.get("projection").as<string>();

	const bool keepChunks = args.has("keep-chunks");
	const bool noChunking = args.has("no-chunking");
	const bool noIndexing = args.has("no-indexing");

	Options options;
	options.source = source;
	options.outdir = outdir;
	options.method = method;
	options.encoding = encoding;
	options.chunkMethod = chunkMethod;
	options.attributes = attributes;
	options.generatePage = generatePage;
	options.pageName = pageName;
	options.pageTitle = pageTitle;
	options.projection = projection;

	options.keepChunks = keepChunks;
	options.noChunking = noChunking;
	options.noIndexing = noIndexing;

	return options;
}

struct Curated{
	string name;
	vector<Source> files;
};
Curated curateSources(const vector<string> &paths) {

	Curated curated;

	vector<string> expanded;
	for (auto && path : paths) {
		if (fs::is_directory(path)) {
			for (auto && entry : fs::directory_iterator(path)) {
				const string str = entry.path().string();
				if (iEndsWith(str, "las") || iEndsWith(str, "laz")) {
					expanded.push_back(str);
				}
			}
		}
		else if (fs::is_regular_file(path)) {
			if (iEndsWith(path, "las") || iEndsWith(path, "laz")) {
				expanded.push_back(path);
			}
		}

		if (curated.name.empty()) {
			curated.name = fs::path(path).stem().string();
		}
	}

	cout << "#paths: " << expanded.size() << endl;

	auto &sources = curated.files;
	sources.reserve(expanded.size());

	mutex mtx;
	constexpr auto parallel = std::execution::par;
	for_each(parallel, expanded.begin(), expanded.end(), [&mtx, &sources](string path) {

		const auto header = loadLasHeader(path);

		Source source;
		source.path = path;
		source.min = { header.min.x, header.min.y, header.min.z };
		source.max = { header.max.x, header.max.y, header.max.z };
		source.numPoints = header.numPoints;
		source.filesize = fs::file_size(path);

		lock_guard<mutex> lock(mtx);
		sources.push_back(source);
	});

	return curated;
}

struct Stats {
	Vector3 min = { Infinity , Infinity , Infinity };
	Vector3 max = { -Infinity , -Infinity , -Infinity };
	int64_t totalBytes = 0;
	int64_t totalPoints = 0;
};

Stats computeStats(const vector<Source> &sources){
	Stats stats;

	for(auto && source : sources){
		stats.min.x = std::min(stats.min.x, source.min.x);
		stats.min.y = std::min(stats.min.y, source.min.y);
		stats.min.z = std::min(stats.min.z, source.min.z);

		stats.max.x = std::max(stats.max.x, source.max.x);
		stats.max.y = std::max(stats.max.y, source.max.y);
		stats.max.z = std::max(stats.max.z, source.max.z);

		stats.totalPoints += source.numPoints;
		stats.totalBytes += source.filesize;
	}

	const double cubeSize = (stats.max - stats.min).max();
	const Vector3 size = { cubeSize, cubeSize, cubeSize };
	stats.max = stats.min + cubeSize;

#ifdef _DEBUG
	const string strMin = "[" + to_string(stats.min.x) + ", " + to_string(stats.min.y) + ", " + to_string(stats.min.z) + "]";
	const string strMax = "[" + to_string(stats.max.x) + ", " + to_string(stats.max.y) + ", " + to_string(stats.max.z) + "]";
	const string strSize = "[" + to_string(size.x) + ", " + to_string(size.y) + ", " + to_string(size.z) + "]";

	string strTotalFileSize;
	{
		constexpr int64_t KB = 1024;
		constexpr int64_t MB = 1024 * KB;
		constexpr int64_t GB = 1024 * MB;
		constexpr int64_t TB = 1024 * GB;

		if (stats.totalBytes >= TB)	strTotalFileSize = formatNumber(double(stats.totalBytes) / double(TB), 1) + " TB";
		else if (stats.totalBytes >= GB)	strTotalFileSize = formatNumber(double(stats.totalBytes) / double(GB), 1) + " GB";
		else if (stats.totalBytes >= MB)	strTotalFileSize = formatNumber(double(stats.totalBytes) / double(MB), 1) + " MB";
		else	strTotalFileSize = formatNumber(double(stats.totalBytes), 1) + " bytes";
	}

	cout << "cubicAABB: {\n";
	cout << "	\"min\": " << strMin << ",\n";
	cout << "	\"max\": " << strMax << ",\n";
	cout << "	\"size\": " << strSize << "\n";
	cout << "}\n";

	cout << "#points: " << formatNumber(stats.totalPoints) << endl;
	cout << "total file size: " << strTotalFileSize << endl;
#endif // _DEBUG

	{ //	sanity check
		const bool sizeError = (size.x == 0.0) || (size.y == 0.0) || (size.z == 0);
		if (sizeError) {
			logger::ERROR("invalid bounding box. at least one axis has a size of zero.");

			exit(123);
		}

	}

	return stats;
}

void chunking(const Options& options, const vector<Source>& sources, const string &targetDir, const Stats& stats, State& state, Attributes &outputAttributes) {

	if (options.noChunking) {
		return;
	}

	if (options.chunkMethod == "LASZIP") {

		chunker_countsort_laszip::doChunking(sources, targetDir, stats.min, stats.max, state, outputAttributes);

	} else if (options.chunkMethod == "LAS_CUSTOM") {
	} else if (options.chunkMethod == "SKIP") {

		//	skip chunking

	} else {

		cout << "ERROR: unkown chunk method: " << options.chunkMethod << endl;
		exit(123);

	}
}

void indexing(Options& options, string targetDir, State& state) {

	if (options.noIndexing) {
		return;
	}

	if (options.method == "random") {

		SamplerRandom sampler;
		indexer::doIndexing(targetDir, state, options, sampler);

	} else if (options.method == "poisson") {

		SamplerPoisson sampler;
		indexer::doIndexing(targetDir, state, options, sampler);

	} else if (options.method == "poisson_average") {

		SamplerPoissonAverage sampler;
		indexer::doIndexing(targetDir, state, options, sampler);

	}
}

void createReport(const Options& options, const vector<Source> &sources, const string &targetDir, const Stats& stats, const State& state, double tStart) {
	const double duration = now() - tStart;
	const double throughputMB = (stats.totalBytes / duration) / (1024 * 1024);
	const double throughputP = (double(stats.totalPoints) / double(duration)) / 1'000'000.0;

	constexpr double kb = 1024.0;
	constexpr double mb = 1024.0 * 1024.0;
	constexpr double gb = 1024.0 * 1024.0 * 1024.0;
	double inputSize = 0;
	string inputSizeUnit = "";
	if (stats.totalBytes <= 10.0 * kb) {
		inputSize = stats.totalBytes / kb;
		inputSizeUnit = "KB";
	}
	else if (stats.totalBytes <= 10.0 * mb) {
		inputSize = stats.totalBytes / mb;
		inputSizeUnit = "MB";
	}
	else if (stats.totalBytes <= 10.0 * gb) {
		inputSize = stats.totalBytes / gb;
		inputSizeUnit = "GB";
	}
	else {
		inputSize = stats.totalBytes / gb;
		inputSizeUnit = "GB";
	}

	cout << endl;
	cout << "=======================================" << endl;
	cout << "=== STATS                              " << endl;
	cout << "=======================================" << endl;

	cout << "#points:               " << formatNumber(stats.totalPoints) << endl;
	cout << "#input files:          " << formatNumber(sources.size()) << endl;
	cout << "sampling method:       " << options.method << endl;
	cout << "chunk method:          " << options.chunkMethod << endl;
	cout << "input file size:       " << formatNumber(inputSize, 1) << inputSizeUnit << endl;
	cout << "duration:              " << formatNumber(duration, 3) << "s" << endl;
	cout << "throughput (MB/s)      " << formatNumber(throughputMB) << "MB" << endl;
	cout << "throughput (points/s)  " << formatNumber(throughputP, 1) << "M" << endl;
	cout << "output location:       " << targetDir << endl;

	for (const auto [key, value] : state.values) {
		cout << key << ": \t" << value << endl;
	}


}

void generatePage(const string &exePath, const string &pagedir, const string &pagename) {
	const string templateDir = exePath + "/resources/page_template";
	const string templateSourcePath = templateDir + "/viewer_template.html";

	const string pageTargetPath = pagedir + "/" + pagename + ".html";

	try{
		fs::copy(templateDir, pagedir, fs::copy_options::overwrite_existing | fs::copy_options::recursive);
	} catch (const std::exception & e) {
		const string msg = e.what();
		logger::ERROR(msg);
	}

	fs::remove(pagedir + "/viewer_template.html");

	{ //	configure page template
		const string strTemplate = readFile(templateSourcePath);

		const string strPointcloudTemplate =
		R"V0G0N(

		Potree.loadPointCloud("<!-- URL -->", "<!-- NAME -->", e => {
			let scene = viewer.scene;
			let pointcloud = e.pointcloud;

			let material = pointcloud.material;
			material.size = 1;
			material.pointSizeType = Potree.PointSizeType.ADAPTIVE;
			material.shape = Potree.PointShape.SQUARE;
			material.activeAttributeName = "rgba";

			scene.addPointCloud(pointcloud);

			viewer.fitToScreen();
		});

		)V0G0N";

		const string url = "./pointclouds/" + pagename + "/metadata.json";

		string strPointcloud = stringReplace(strPointcloudTemplate, "<!-- URL -->", url);
		strPointcloud = stringReplace(strPointcloud, "<!-- NAME -->", pagename);

		const string strPage = stringReplace(strTemplate, "<!-- INCLUDE POINTCLOUD -->", strPointcloud);


		writeFile(pageTargetPath, strPage);

	}

}

#ifdef DEBUG_STUFF
#include "HierarchyBuilder.h"
#endif // DEBUG_STUFF

int main(int argc, char** argv) {

#ifdef DEBUG_STUFF
	{

		string hierarchyDir = "D:/dev/pointclouds/Riegl/retz_converted/.hierarchyChunks";
		constexpr int hierarchyStepSize = 4;

		HierarchyBuilder builder(hierarchyDir, hierarchyStepSize);
		builder.build();

		return 0;
	}
#endif // DEBUG_STUFF

	const double tStart = now();

	const auto exePath = fs::canonical(fs::absolute(argv[0])).parent_path().string();

#ifdef _DEBUG
	launchMemoryChecker(0.1);
#endif // _DEBUG
	const auto cpuData = getCpuData();

	cout << "#threads: " << cpuData.numProcessors << endl;

	auto options = parseArguments(argc, argv);

	auto [name, sources] = curateSources(options.source);
	if (options.name.empty()) {
		options.name = name;
	}

	auto outputAttributes = computeOutputAttributes(sources, options.attributes);
	cout << toString(outputAttributes);

	const auto stats = computeStats(sources);
	string targetDir = options.outdir;
	if (options.generatePage) {

		generatePage(exePath, targetDir, options.pageName);

		targetDir += "/pointclouds/" + options.pageName;
	}
	cout << "target directory: '" << targetDir << "'" << endl;
	fs::create_directories(targetDir);
	logger::addOutputFile(targetDir + "/log.txt");

	State state;
	state.pointsTotal = stats.totalPoints;
	state.bytesProcessed = stats.totalBytes;

	auto monitor = make_shared<Monitor>(&state);
	monitor->start();


	{ //	this is the real important stuff

		chunking(options, sources, targetDir, stats, state, outputAttributes);

		indexing(options, targetDir, state);

	}

	monitor->stop();

	createReport(options, sources, targetDir, stats, state, tStart);


	return 0;
}