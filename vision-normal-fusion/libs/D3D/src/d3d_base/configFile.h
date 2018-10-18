
#ifndef CONFIGFILE_H
#define CONFIGFILE_H

#include <map>
#include <string>

namespace D3D
{
    using std::map;
    using std::string;

    class ConfigFile {
    public:
        ConfigFile(const string& fileName);

        string get(const string& configParameter);
        int getAsInt(const string& configParameter);
        float getAsFloat(const string& configParameter);

        bool isFileRead();

    private:

        bool fileRead;

        void parseLine(string& line);

        map<string, string> configEntries;
    };
}

#endif // CONFIGFILE_H
