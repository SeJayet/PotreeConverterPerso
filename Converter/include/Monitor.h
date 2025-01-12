#pragma once

#include <thread>
#include <map>

#include "converter_utils.h"

using std::thread;
using std::map;

struct Monitor {
	thread t;
	bool stopRequested = false;
	State* state = nullptr;
	map<string, string> messages;

	Monitor(State* state){
		this->state = state;
	}

	void _print(){
		const auto ram = getMemoryData();
		const auto CPU = getCpuData();
		constexpr double GB = 1024.0 * 1024.0 * 1024.0;

		const double throughput = (this->state->pointsProcessed / this->state->duration) / 1'000'000.0;

		const double progressPass = 100.0 * this->state->progress();
		const double progressTotal = (100.0 * (this->state->currentPass - 1) + progressPass) / this->state->numPasses;

		const string strProgressPass = formatNumber(progressPass) + "%";
		const string strProgressTotal = formatNumber(progressTotal) + "%";
		const string strTime = formatNumber(now()) + "s";
		const string strDuration = formatNumber(this->state->duration) + "s";
		const string strThroughput = formatNumber(throughput) + "MPs";

		const string strRAM = formatNumber(double(ram.virtual_usedByProcess) / GB, 1)
			+ "GB (highest " + formatNumber(double(ram.virtual_usedByProcess_max) / GB, 1) + "GB)";
		const string strCPU = formatNumber(CPU.usage) + "%";

		stringstream ss;
		ss << "[" << strProgressTotal << ", " << strTime << "], "
			<< "[" << this->state->name << ": " << strProgressPass 
			<< ", duration: " << strDuration 
			<< ", throughput: " << strThroughput << "]"
			<< "[RAM: " << strRAM << ", CPU: " << strCPU << "]" << endl;

		cout << ss.str() << std::flush;

	}

	void start(){

		Monitor* _this = this;
		this->t = thread([_this]() {
			std::this_thread::sleep_for(std::chrono::seconds(1));
			
			cout << endl;

			while (!_this->stopRequested) {

				_this->_print();

				std::this_thread::sleep_for(std::chrono::seconds(1));
			}

		});

	}

	void print(string key, string message){

	}

	void stop() {

		stopRequested = true;

		t.join();
	}

};