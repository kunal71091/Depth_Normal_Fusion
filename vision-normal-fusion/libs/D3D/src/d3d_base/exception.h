#ifndef EXCEPTION_H
#define EXCEPTION_H

#include <exception>
#include <string>
#include <sstream>

using std::stringstream;

namespace D3D
{

#ifdef _MSC_VER
	#define D3D_THROW_EXCEPTION(message) throw D3D::Exception(__FUNCSIG__, (message));
#else
    #define D3D_THROW_EXCEPTION(message) throw D3D::Exception(__PRETTY_FUNCTION__, (message));
#endif

    class Exception : public std::exception
    {
    public:
        Exception(std::string& message)
            : message(message)
        { }

        Exception(const char* place, const char* message)
        {
            stringstream messageStream;
            messageStream  << place << " : " << message;
            this->message = messageStream.str();
        }

        Exception(const char* fileName, int lineNumber, const char* place, const char* message)
        {
            stringstream messageStream;
            messageStream << fileName << "(" << lineNumber << ") " << place << " : " << message;
            this->message = messageStream.str();
        }

        virtual ~Exception() throw() { }

        virtual const char* what() const throw()
        {
            return message.c_str();
        }

    protected:
        std::string message;
    };
}

#endif // EXCEPTION_H
