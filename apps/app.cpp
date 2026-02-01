#include <atomic>
#include <csignal>
#include <iostream>
#include <mvio/config.h>
#include <mvio/pipeline/Pipeline.h>
#include <opencv2/core.hpp>

std::atomic<bool> g_stop_signal{false};

void signal_handler(int signum) {
	if (signum == SIGINT) {
		std::cout << "\nReceived SIGINT. Stopping pipeline..." << std::endl;
		g_stop_signal = true;
	}
}

int main() {
	std::signal(SIGINT, signal_handler);
	cv::setRNGSeed(0);

	Config cfg;
	cfg.load("configs/config.yaml");

	try {
		mvio::Pipeline pipeline(cfg, g_stop_signal);
		pipeline.run();
	} catch (const std::exception &e) {
		std::cerr << "Fatal error: " << e.what() << std::endl;
		return 1;
	}

	return 0;
}
