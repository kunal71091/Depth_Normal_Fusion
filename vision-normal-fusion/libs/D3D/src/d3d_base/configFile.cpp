
#include "configFile.h"
#include <d3d_base/exception.h>

#include <fstream>
#include <sstream>
#include <iostream>
#include <boost/algorithm/string.hpp>

using namespace D3D;
using std::ifstream;
using std::getline;
using std::istringstream;
using boost::algorithm::trim;

ConfigFile::ConfigFile(const string& fileName) {

    fileRead = false;

    ifstream file;
    file.open(fileName.c_str());

    if (!file.is_open())
        return;

    while (!file.eof()) {
        string line;
        getline(file, line);
        parseLine(line);
    }

    fileRead = true;
}

void ConfigFile::parseLine(string& line) {

    int commentPos = (int) line.find('#');

    // nothing interesting before the comment
    if (commentPos != -1 && commentPos < 3)
        return;

    // remove commented part
    if (commentPos != -1)
    {
        line = line.substr(0, commentPos);
    }
    int eqPos = (int) line.find('=');
    if (eqPos == -1)
        return;

    string before = line.substr(0, eqPos);
    string after = line.substr(eqPos+1, line.length()-eqPos-1);

    trim(before);
    trim(after);

    // if both strings are not emtpy add the pair to the hash table
    if (before.length() > 0 && after.length() > 0)
        this->configEntries[before] = after;
}

string ConfigFile::get(const string& configParameter)
{
    if (configEntries.count(configParameter) != 0)
    {
        return configEntries[configParameter];
    }
    else
    {
        return string();
    }
}

int ConfigFile::getAsInt(const string& configParameter)
{
    std::string paramStr = get(configParameter);
    if (paramStr.empty())
    {
        std::stringstream strStr;
        strStr << "Parameter " << configParameter << " is not defined in config file.";
        D3D_THROW_EXCEPTION(strStr.str().c_str());
    }
    return atoi(paramStr.c_str());
}

float ConfigFile::getAsFloat(const string& configParameter)
{
    std::string paramStr = get(configParameter);
    if (paramStr.empty())
    {
        std::stringstream strStr;
        strStr << "Parameter " << configParameter << " is not defined in config file.";
        D3D_THROW_EXCEPTION(strStr.str().c_str());
    }
    return (float) atof(paramStr.c_str());
}
bool ConfigFile::isFileRead() {
    return fileRead;
}
