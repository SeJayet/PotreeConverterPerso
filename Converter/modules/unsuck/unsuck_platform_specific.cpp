#include "unsuck.hpp"

#ifdef _WIN32
	#include "TCHAR.h"
	#include "pdh.h"
	#include "windows.h"
	#include "psapi.h"

// see https://stackoverflow.com/questions/63166/how-to-determine-cpu-and-memory-consumption-from-inside-a-process
MemoryData getMemoryData() {

	MemoryData data;

	{
		MEMORYSTATUSEX memInfo;
		memInfo.dwLength = sizeof(MEMORYSTATUSEX);
		GlobalMemoryStatusEx(&memInfo);

		data.virtual_total = memInfo.ullTotalPageFile;
		data.virtual_used = memInfo.ullTotalPageFile - memInfo.ullAvailPageFile;

		data.physical_total = memInfo.ullTotalPhys;
		data.physical_used = memInfo.ullTotalPhys - memInfo.ullAvailPhys;

	}

	{
		PROCESS_MEMORY_COUNTERS_EX pmc;
		GetProcessMemoryInfo(GetCurrentProcess(), reinterpret_cast<PROCESS_MEMORY_COUNTERS*>(&pmc), sizeof(pmc));
		const SIZE_T virtualMemUsedByMe = pmc.PrivateUsage;
		const SIZE_T physMemUsedByMe = pmc.WorkingSetSize;

		static size_t virtualUsedMax = 0;
		static size_t physicalUsedMax = 0;

		virtualUsedMax = max(virtualMemUsedByMe, virtualUsedMax);
		physicalUsedMax = max(physMemUsedByMe, physicalUsedMax);

		data.virtual_usedByProcess = virtualMemUsedByMe;
		data.virtual_usedByProcess_max = virtualUsedMax;
		data.physical_usedByProcess = physMemUsedByMe;
		data.physical_usedByProcess_max = physicalUsedMax;
	}

	return data;
}


#ifdef _DEBUG
void launchMemoryChecker(double checkInterval) {

	const auto interval = std::chrono::milliseconds(int64_t(checkInterval * 1000));

	thread t([interval]() {

		while (true) {
			const auto memdata = getMemoryData();
			std::this_thread::sleep_for(interval);
		}

	});
	t.detach();

}
#endif // _DEBUG

static ULARGE_INTEGER lastCPU, lastSysCPU, lastUserCPU;
static int numProcessors;
static HANDLE self;
static bool initialized = false;

void init() {
	FILETIME ftime, fsys, fuser;

	numProcessors = std::thread::hardware_concurrency();

	GetSystemTimeAsFileTime(&ftime);
	memcpy(&lastCPU, &ftime, sizeof(FILETIME));

	self = GetCurrentProcess();
	GetProcessTimes(self, &ftime, &ftime, &fsys, &fuser);
	memcpy(&lastSysCPU, &fsys, sizeof(FILETIME));
	memcpy(&lastUserCPU, &fuser, sizeof(FILETIME));

	initialized = true;
}

CpuData getCpuData() {
	FILETIME ftime, fsys, fuser;
	ULARGE_INTEGER now, sys, user;
	double percent;

	if (!initialized) {
		init();
	}

	GetSystemTimeAsFileTime(&ftime);
	memcpy(&now, &ftime, sizeof(FILETIME));

	GetProcessTimes(self, &ftime, &ftime, &fsys, &fuser);
	memcpy(&sys, &fsys, sizeof(FILETIME));
	memcpy(&user, &fuser, sizeof(FILETIME));
	percent = (sys.QuadPart - lastSysCPU.QuadPart) +
		(user.QuadPart - lastUserCPU.QuadPart);
	percent /= (now.QuadPart - lastCPU.QuadPart);
	percent /= numProcessors;
	lastCPU = now;
	lastUserCPU = user;
	lastSysCPU = sys;

	CpuData data;
	data.numProcessors = numProcessors;
	data.usage = percent * 100.0;

	return data;
}

#elif defined(__linux__)

// see https://stackoverflow.com/questions/63166/how-to-determine-cpu-and-memory-consumption-from-inside-a-process

#include "sys/types.h"
#include "sys/sysinfo.h"

#include "stdlib.h"
#include "stdio.h"
#include "string.h"

int parseLine(char* line){
	// This assumes that a digit will be found and the line ends in " Kb".
	int i = strlen(line);
	const char* p = line;

	while (*p < '0' || *p > '9'){
		p++;
	}

	line[i - 3] = '\0';
	i = atoi(p);

	return i;
}

int64_t getVirtualMemoryUsedByProcess(){ //Note: this value is in KB!
	FILE* file = fopen("/proc/self/status", "r");
	int64_t result = -1;
	char line[128];

	while (fgets(line, 128, file) != NULL){
		if (strncmp(line, "VmSize:", 7) == 0){
			result = parseLine(line);
			break;
		}
	}
	fclose(file);

	result = result * 1024;

	return result;
}

int64_t getPhysicalMemoryUsedByProcess(){ //Note: this value is in KB!
	FILE* file = fopen("/proc/self/status", "r");
	int64_t result = -1;
	char line[128];

	while (fgets(line, 128, file) != NULL){
		if (strncmp(line, "VmRSS:", 6) == 0){
			result = parseLine(line);
			break;
		}
	}
	fclose(file);

	result = result * 1024;

	return result;
}


MemoryData getMemoryData() {

	struct sysinfo memInfo;

	sysinfo (&memInfo);
	int64_t totalVirtualMem = memInfo.totalram;
	totalVirtualMem += memInfo.totalswap;
	totalVirtualMem *= memInfo.mem_unit;

	int64_t virtualMemUsed = memInfo.totalram - memInfo.freeram;
	virtualMemUsed += memInfo.totalswap - memInfo.freeswap;
	virtualMemUsed *= memInfo.mem_unit;

	int64_t totalPhysMem = memInfo.totalram;
	totalPhysMem *= memInfo.mem_unit;

	long long physMemUsed = memInfo.totalram - memInfo.freeram;
	physMemUsed *= memInfo.mem_unit;

	int64_t virtualMemUsedByMe = getVirtualMemoryUsedByProcess();
	int64_t physMemUsedByMe = getPhysicalMemoryUsedByProcess();


	MemoryData data;

	static int64_t virtualUsedMax = 0;
	static int64_t physicalUsedMax = 0;

	virtualUsedMax = std::max(virtualMemUsedByMe, virtualUsedMax);
	physicalUsedMax = std::max(physMemUsedByMe, physicalUsedMax);

	{
		data.virtual_total = totalVirtualMem;
		data.virtual_used = virtualMemUsed;
		data.physical_total = totalPhysMem;
		data.physical_used = physMemUsed;

	}

	{
		data.virtual_usedByProcess = virtualMemUsedByMe;
		data.virtual_usedByProcess_max = virtualUsedMax;
		data.physical_usedByProcess = physMemUsedByMe;
		data.physical_usedByProcess_max = physicalUsedMax;
	}


	return data;
}


#ifdef _DEBUG
void launchMemoryChecker(double checkInterval) {

	auto interval = std::chrono::milliseconds(int64_t(checkInterval * 1000));

	thread t([interval]() {

		while (true) {
			const auto memdata = getMemoryData();
			std::this_thread::sleep_for(interval);
		}

	});
	t.detach();

}
#endif // _DEBUG

static int numProcessors;
static bool initialized = false;
static unsigned long long lastTotalUser, lastTotalUserLow, lastTotalSys, lastTotalIdle;

void init() {
	numProcessors = std::thread::hardware_concurrency();

	FILE* file = fopen("/proc/stat", "r");
	fscanf(file, "cpu %llu %llu %llu %llu", &lastTotalUser, &lastTotalUserLow, &lastTotalSys, &lastTotalIdle);
	fclose(file);

	initialized = true;
}

double getCpuUsage(){
	double percent;
	FILE* file;
	unsigned long long totalUser, totalUserLow, totalSys, totalIdle, total;

	file = fopen("/proc/stat", "r");
	fscanf(file, "cpu %llu %llu %llu %llu", &totalUser, &totalUserLow, &totalSys, &totalIdle);
	fclose(file);

	if (totalUser < lastTotalUser || totalUserLow < lastTotalUserLow ||
		totalSys < lastTotalSys || totalIdle < lastTotalIdle){
		//Overflow detection. Just skip this value.
		percent = -1.0;
	}else{
		total = (totalUser - lastTotalUser)
			+ (totalUserLow - lastTotalUserLow)
			+ (totalSys - lastTotalSys);
		percent = total;
		total += (totalIdle - lastTotalIdle);
		percent /= total;
		percent *= 100;
	}

	lastTotalUser = totalUser;
	lastTotalUserLow = totalUserLow;
	lastTotalSys = totalSys;
	lastTotalIdle = totalIdle;

	return percent;
}

CpuData getCpuData() {

	if (!initialized) {
		init();
	}

	CpuData data;
	data.numProcessors = numProcessors;
	data.usage = getCpuUsage();

	return data;
}


#endif