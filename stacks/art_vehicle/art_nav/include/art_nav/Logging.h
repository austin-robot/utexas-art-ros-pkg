////////////////////////////////////////////////////////////////////////////////
//
// $Id$
//
////////////////////////////////////////////////////////////////////////////////
#ifndef LOGGING_H
#define LOGGING_H

#include <fstream>
#include <iostream>

///@brief Logging class to give message notices
///
///@par Usage:
///create a temporary instantiation of the object with the
///constructor arguments specifying the severity of the message
///Then just use it as if it were a stream to
///input the message
///
///@note This class should only ever be used as a temporary class.
///
///@par Example:
///Log(7,"Memory manager") << (mTotalMem - mUsedMem) << "memory left";
///
class logc
{

protected:
        static std::ofstream mStream;
        static int mCoutPriority;
		int mPriority;
public:
        logc(int Priority);

        virtual ~logc();

        ///Works just like an iostream operator
        ///
        template<typename T>
        logc& operator<<(const T Add)
        {
                mStream << Add;

                if(mPriority >= mCoutPriority)
                {
					std::cout << Add;
                }
                
                return *this;
        }

        void SetCoutLevel(int Priority);
};


#endif//end
