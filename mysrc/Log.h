/*
 * Log.h
 *
 *  Created on: Jun 22, 2013
 *      Author: Tuan A.Nguyen
 */

#ifndef LOG_H_
#define LOG_H_

#include <string>
#include <sstream>
#include <fstream>

namespace tlib {

class Log {

	// The file to keep logs
	std::ofstream f;

	// Path and file name
	std::string path, file;

	// If it is open
	bool opened;

	// Write a message
	bool write(std::string msg) {
		if (!is_open())
			return false;
		f<<msg<<std::endl;
		return true;
	}

public:

	// Constructor
	Log() {
		opened = false;
	}

	virtual ~Log() {
		if (is_open()) {
			f.close();
		}
	}

	// Set the path and file name
	void set_path(std::string path) {
		if (!is_open())
			this->path = path;
	}

	void set_file(std::string file) {
		if (!is_open())
			this->file = file;
	}

	// Open the file
	bool open() {
		if (is_open())
			return true;

		std::string s(path);
		if (path.at(path.length()-1) != '/')
			s += '/';
		s += file;
		std::cout<<"s = "<<s<<std::endl;
		f.open(s.c_str(), std::ofstream::out | std::ofstream::app);
		if (!f.good()) {
			f.close();
			return false;
		}

		// Mark file as open
		opened = true;

		return true;
	}

	// Check if file is opened
	bool is_open() {
		return opened;
	}

	// Check a logical condition in a file at a particular line. Two cases:
	// + True: nothing happens
	// + False: write into the log, and exit
	void assertTrue(bool condition, char file[], int line) {
		if (!is_open())
			return;

		if (!condition) {
			std::stringstream ss;
			ss<<line;
			std::string line_str = ss.str();
			std::string msg = std::string("Assertion fails! ") + std::string(file) + std::string(": ") + line_str;
			f<<msg<<std::endl;

			// Stop the program
			exit(1);
		}
	}

	// Write a message to the log file
	friend Log& operator<<(Log& l, std::string msg) {
		if (l.is_open())
			l.write(msg);
		return l;
	}
};

} /* namespace tlib */
#endif /* LOG_H_ */
